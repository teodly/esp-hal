#![cfg_attr(docsrs, procmacros::doc_replace(
    "dac1_pin" => {
        cfg(esp32) => "let dac1_pin = peripherals.GPIO25;",
        cfg(esp32s2) => "let dac1_pin = peripherals.GPIO17;"
    }
))]
//! # Digital to Analog Converter (DAC)
//!
//! ## Overview
//! Espressif devices usually have multiple DAC channels. Each DAC channel can
//! convert the digital value 0~255 to the analog voltage 0~Vref (The reference
//! voltage 'Vref' here is input from an input pin)
//!
//! The DAC peripheral supports outputting analog signal in the multiple ways.
//!
//! Two 8-bit DAC channels are available.
//!
//! ## Configuration
//! Developers can choose the  DAC channel they want to use based on the GPIO
//! pin assignments for each channel.
//!
//! ## Examples
//! ### Write a value to a DAC channel
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::analog::dac::Dac;
//! # use esp_hal::delay::Delay;
//! # use embedded_hal::delay::DelayNs;
//! # {dac1_pin}
//! let mut dac1 = Dac::new(peripherals.DAC1, dac1_pin);
//!
//! let mut delay = Delay::new();
//!
//! let mut voltage_dac1 = 200u8;
//!
//! // Change voltage on the pins using write function:
//! loop {
//!     voltage_dac1 = voltage_dac1.wrapping_add(1);
//!     dac1.write(voltage_dac1);
//!
//!     delay.delay_ms(50u32);
//! }
//! # }
//! ```

use crate::gpio::AnalogPin;

#[cfg(esp32s2)]
pub mod continuous_dma {
    //! ESP32-S2 DAC continuous output backed by SPI3 DMA (no_std, esp-hal).
    //!
    //! ## Hardware notes (ESP32-S2)
    //! - The DAC “continuous” engine is driven by the APB_SARADC digital controller.
    //! - The data source for DAC DMA is SPI3 DMA TX (no SPI pins involved).
    //! - The DAC digital controller shares clocking/dividers with the ADC digital controller. This
    //!   is typically compatible with ADC *oneshot* reads, but may interfere with ADC
    //!   digital/continuous modes.
    //!
    //! ## Current API surface
    //! This module intentionally mirrors the I2S TX DMA ergonomics:
    //! - you start a circular DMA transfer with `write_dma_circular()`
    //! - then you refill the ring buffer using the returned `DmaTransferTxCircular` handle
    //!   (`available()`, `push_with()`, etc).
    //!
    //! This is designed for mono output. SPI3 is assumed dedicated to DAC DMA.

    use enumset::EnumSet;

    use crate::{
        Blocking,
        DriverMode,
        analog::dac::Dac,
        dma::{
            Channel,
            ChannelTx,
            DescriptorChain,
            DmaChannelFor,
            DmaDescriptor,
            DmaError,
            DmaPeripheral,
            DmaTransferTx,
            DmaTransferTxCircular,
            DmaTxInterrupt,
            InterruptAccess,
            PeripheralTxChannel,
            ReadBuffer,
            RegisterAccess,
            TxRegisterAccess,
            dma_private::{DmaSupport, DmaSupportTx},
        },
        peripherals::{APB_SARADC, DAC1, GPIO17, SENS, SPI3},
        system::{Peripheral, PeripheralGuard},
    };

    /// Errors returned by the DAC continuous DMA driver.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Error {
        /// The requested frequency cannot be represented with the available dividers.
        UnsupportedFrequency,
        /// DMA configuration or runtime error.
        Dma(DmaError),
    }

    impl From<DmaError> for Error {
        fn from(e: DmaError) -> Self {
            Error::Dma(e)
        }
    }

    /// Configuration for DAC continuous output.
    #[derive(Debug, Clone, Copy)]
    pub struct Config {
        /// DAC conversion frequency in Hz (bytes/s for mono).
        pub freq_hz: u32,
        /// Whether to invert the DAC digital controller clock.
        pub invert_clock: bool,
    }

    impl Config {
        /// Create a new configuration for mono continuous DAC output.
        ///
        /// `freq_hz` is the DAC conversion frequency (i.e. bytes/s for mono).
        pub const fn new(freq_hz: u32) -> Self {
            Self {
                freq_hz,
                invert_clock: true, // matches IDF default behavior for ESP32-S2
            }
        }

        /// Configure whether to invert the DAC digital controller clock.
        pub const fn with_invert_clock(mut self, invert: bool) -> Self {
            self.invert_clock = invert;
            self
        }
    }

    #[derive(Clone, Copy)]
    struct ClkDiv {
        integer: u32,
        denominator: u32,
        numerator: u32,
    }

    #[inline(always)]
    fn sub_abs(a: u32, b: u32) -> u32 {
        if a > b { a - b } else { b - a }
    }

    // Port of `hal_utils_calc_clk_div_frac_accurate()` (IDF) for no_std usage.
    fn calc_clk_div_frac_accurate(
        src_freq_hz: u32,
        exp_freq_hz: u32,
        min_integ: u32,
        max_integ: u32,
        max_fract: u32,
    ) -> Option<ClkDiv> {
        if exp_freq_hz == 0 || max_fract <= 2 {
            return None;
        }

        let mut div_denom: u32 = 2;
        let mut div_numer: u32 = 0;
        let mut div_integ: u32 = src_freq_hz / exp_freq_hz;
        let freq_error: u32 = src_freq_hz % exp_freq_hz;

        if freq_error != 0 {
            // Carry bit if the decimal is greater than 1.0 - 1.0 / ((max_fract - 1) * 2)
            let threshold =
                exp_freq_hz.saturating_sub((exp_freq_hz / (max_fract - 1)).saturating_mul(2));
            if freq_error < threshold {
                // Search the closest fraction (O(n), but small: max_fract=64)
                let mut min_sub: u32 = u32::MAX;
                let mut best_a: u32 = 2;
                let mut best_b: u32 = 0;

                let mut a = 2;
                while min_sub != 0 && a < max_fract {
                    // b = round(a * freq_error / exp_freq_hz)
                    let b = (a.saturating_mul(freq_error).saturating_add(exp_freq_hz / 2))
                        / exp_freq_hz;
                    let s = sub_abs(exp_freq_hz.saturating_mul(b), freq_error.saturating_mul(a));
                    if s < min_sub {
                        min_sub = s;
                        best_a = a;
                        best_b = b;
                    }
                    a += 1;
                }

                div_denom = best_a;
                div_numer = best_b;
            } else {
                div_integ = div_integ.saturating_add(1);
            }
        }

        if div_integ < min_integ || div_integ >= max_integ || div_integ == 0 {
            return None;
        }

        Some(ClkDiv {
            integer: div_integ,
            denominator: div_denom,
            numerator: div_numer,
        })
    }

    /// DAC continuous output driver (mono, DAC1 / GPIO17) using SPI3 DMA TX.
    pub struct DacContinuousTx<'d, Dm>
    where
        Dm: DriverMode,
    {
        _dac: Dac<'d, DAC1<'d>>,
        _spi3_guard: PeripheralGuard,
        _spi3: SPI3<'d>,
        tx_channel: ChannelTx<Dm, PeripheralTxChannel<SPI3<'d>>>,
        tx_chain: DescriptorChain,
        cfg: Config,
    }

    impl<'d> DacContinuousTx<'d, Blocking> {
        /// Create a mono continuous DAC output on DAC1 (GPIO17), backed by SPI3 DMA.
        ///
        /// - `spi3` is consumed and reserved for DAC DMA usage.
        /// - `dma` must be `DMA_SPI3`.
        /// - `descriptors` are the DMA descriptors for circular transfers.
        pub fn new(
            dac1: DAC1<'d>,
            dac1_pin: GPIO17<'d>,
            spi3: SPI3<'d>,
            dma: impl DmaChannelFor<SPI3<'d>>,
            cfg: Config,
            descriptors: &'static mut [DmaDescriptor],
        ) -> Result<Self, Error> {
            let _dac = Dac::new(dac1, dac1_pin);

            // Ensure DAC is in "pad source" mode (disable CW generator path for this channel).
            <DAC1<'d> as super::Instance>::set_pad_source();

            // Ensure the DMA channel matches the peripheral
            let channel = Channel::new(dma.degrade());
            channel.runtime_ensure_compatible(&spi3);

            // Keep SPI3 peripheral clock enabled for as long as the DAC continuous DMA driver
            // lives. On ESP32-S2 the DAC DMA backend uses SPI3 DMA registers (and SPI
            // bus clocking matters).
            let spi3_guard = PeripheralGuard::new(Peripheral::Spi3);

            // NOTE: We keep SPI3 token to prevent other users from touching it.
            // We only use SPI3 as a DMA data source for the DAC controller.

            let mut this = Self {
                _dac,
                _spi3_guard: spi3_guard,
                _spi3: spi3,
                tx_channel: channel.tx,
                tx_chain: DescriptorChain::new(descriptors),
                cfg,
            };

            this.configure_clock_and_enable_dma(cfg.freq_hz, false)?;
            Ok(this)
        }
    }

    impl<'d, Dm> DacContinuousTx<'d, Dm>
    where
        Dm: DriverMode,
    {
        #[inline(always)]
        fn dac_digi_enable_dma(enable: bool) {
            // dac_ll_digi_enable_dma(enable):
            // SENS.sar_dac_ctrl1.dac_dig_force = enable;
            // APB_SARADC.apb_dac_ctrl.apb_dac_trans = enable;
            SENS::regs()
                .sar_dac_ctrl1()
                .modify(|_, w| w.dac_dig_force().bit(enable));
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.trans().bit(enable));
        }

        #[inline(always)]
        fn dac_digi_clk_inv(enable: bool) {
            SENS::regs()
                .sar_dac_ctrl1()
                .modify(|_, w| w.dac_clk_inv().bit(enable));
        }

        #[inline(always)]
        fn dac_digi_set_convert_mode(is_alternate: bool) {
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.alter_mode().bit(is_alternate));
        }

        #[inline(always)]
        fn dac_digi_set_trigger_interval(interval: u32) {
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| unsafe { w.timer_target().bits(interval as u16) });
        }

        #[inline(always)]
        fn dac_digi_trigger_output(enable: bool) {
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.timer_en().bit(enable));
        }

        #[inline(always)]
        fn dac_digi_fifo_reset() {
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.reset_fifo().set_bit());
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.reset_fifo().clear_bit());
        }

        #[inline(always)]
        fn dac_digi_reset() {
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.rst().set_bit());
            APB_SARADC::regs()
                .apb_dac_ctrl()
                .modify(|_, w| w.rst().clear_bit());
        }

        fn configure_clock_and_enable_dma(
            &mut self,
            freq_hz: u32,
            is_alternate: bool,
        ) -> Result<(), Error> {
            // This is a no_std port of the relevant parts of:
            // - dac_dma_periph_init()
            // - s_dac_dma_periph_set_clock()
            //
            // We use APB clock only for now (clk_sel = 2).

            if freq_hz == 0 {
                return Err(Error::UnsupportedFrequency);
            }

            let apb_hz = crate::clock::Clocks::get().apb_clock.as_hz() as u32;
            let trans_freq_hz = freq_hz.saturating_mul(if is_alternate { 2 } else { 1 });

            let total_div = apb_hz / trans_freq_hz;
            if total_div < 2 {
                return Err(Error::UnsupportedFrequency);
            }

            let interval: u32 = if total_div < 256 {
                1
            } else if total_div < 8192 {
                total_div / 2
            } else {
                4095
            };

            if interval.saturating_mul(256) <= total_div {
                return Err(Error::UnsupportedFrequency);
            }

            let src_freq_hz = apb_hz / interval;
            let div = calc_clk_div_frac_accurate(src_freq_hz, trans_freq_hz, 1, 257, 64)
                .ok_or(Error::UnsupportedFrequency)?;

            // Program clock:
            // adc_ll_digi_controller_clk_div(div.integer - 1, div.denominator, div.numerator)
            // adc_ll_digi_clk_sel(DEFAULT)
            APB_SARADC::regs().clkm_conf().modify(|_, w| unsafe {
                w.clkm_div_num().bits((div.integer - 1) as u8);
                w.clkm_div_b().bits(div.denominator as u8);
                w.clkm_div_a().bits(div.numerator as u8);
                w.clk_sel().bits(2); // ADC_DIGI_CLK_SRC_DEFAULT on ESP32-S2 ll sets clk_sel=2
                w
            });
            APB_SARADC::regs()
                .ctrl()
                .modify(|_, w| w.sar_clk_gated().set_bit());

            // Program DAC digital controller:
            Self::dac_digi_clk_inv(self.cfg.invert_clock);
            Self::dac_digi_set_trigger_interval(interval);
            Self::dac_digi_set_convert_mode(is_alternate);

            // Reset digital controller / FIFO and connect DMA
            Self::dac_digi_reset();
            Self::dac_digi_fifo_reset();
            Self::dac_digi_enable_dma(true);

            Ok(())
        }

        fn start_tx_transfer<'t, TXBUF>(
            &'t mut self,
            words: &'t TXBUF,
            circular: bool,
        ) -> Result<(), Error>
        where
            TXBUF: ReadBuffer,
        {
            let (ptr, len) = unsafe { words.read_buffer() };

            // Prepare DAC digital controller for playback
            //
            // Enable the trigger output *before* starting DMA so the DAC engine is clocked/driven
            // while the SPI3 DMA outlink is being brought up.
            Self::dac_digi_trigger_output(false);
            Self::dac_digi_fifo_reset();
            Self::dac_digi_trigger_output(true);

            // Configure descriptor chain
            self.tx_chain.fill_for_tx(circular, ptr, len)?;

            // Enable EOF interrupt so `DmaTransferTxCircular::available()` can progress.
            self.tx_channel
                .listen_out(EnumSet::only(DmaTxInterrupt::Eof));

            // Prepare SPI3 DMA outlink manually (avoid auto-write-back, which SPI DMA doesn't
            // support) This mirrors `ChannelTx::do_prepare()` but forces
            // `auto_write_back = false`.
            self.tx_channel
                .tx_impl
                .set_burst_mode(crate::dma::BurstConfig::default());
            self.tx_channel.tx_impl.set_descr_burst_mode(true);
            self.tx_channel.tx_impl.set_check_owner(Some(false));
            self.tx_channel.tx_impl.set_auto_write_back(false);

            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

            self.tx_channel.tx_impl.clear_all();
            self.tx_channel.tx_impl.reset();
            self.tx_channel
                .tx_impl
                .set_link_addr(self.tx_chain.first().cast_mut() as u32);
            self.tx_channel
                .tx_impl
                .set_peripheral(DmaPeripheral::Spi3 as u8);

            self.tx_channel.tx_impl.start();

            // On SPI3-DMA-backed DAC output, waiting for the SPI DMA FIFO to become non-empty can
            // deadlock depending on peripheral state/clocking. Instead, start the DAC trigger
            // immediately and rely on descriptor error interrupts to surface failures.
            if self
                .tx_channel
                .pending_out_interrupts()
                .contains(DmaTxInterrupt::DescriptorError)
            {
                return Err(Error::Dma(DmaError::DescriptorError));
            }

            // DAC trigger output is already enabled above (before DMA start).

            Ok(())
        }

        /// Continuously write to the DAC using a circular DMA buffer.
        ///
        /// The returned transfer handle provides `available()`/`push_with()` for refilling.
        pub fn write_dma_circular<'t>(
            &'t mut self,
            words: &'t impl ReadBuffer,
        ) -> Result<DmaTransferTxCircular<'t, Self>, Error>
        where
            Self: DmaSupportTx,
        {
            self.start_tx_transfer(words, true)?;
            Ok(DmaTransferTxCircular::new(self))
        }

        /// Write a single-shot DMA buffer (acyclic).
        pub fn write_dma<'t>(
            &'t mut self,
            words: &'t impl ReadBuffer,
        ) -> Result<DmaTransferTx<'t, Self>, Error>
        where
            Self: DmaSupportTx,
        {
            self.start_tx_transfer(words, false)?;
            Ok(DmaTransferTx::new(self))
        }
    }

    impl<'d, Dm> DmaSupport for DacContinuousTx<'d, Dm>
    where
        Dm: DriverMode,
    {
        type DriverMode = Dm;

        fn peripheral_wait_dma(&mut self, _is_rx: bool, _is_tx: bool) {
            // For one-shot transfers, wait for TotalEof.
            // For circular transfers, users should call `stop()`.
            while !self.tx_channel.is_done() && !self.tx_channel.has_error() {}
        }

        fn peripheral_dma_stop(&mut self) {
            // Stop DAC trigger first to avoid consuming stale data, then stop DMA outlink.
            Self::dac_digi_trigger_output(false);
            self.tx_channel.stop_transfer();

            // Optionally disconnect DMA path (keeps DAC powered):
            Self::dac_digi_enable_dma(false);
        }
    }

    impl<'d, Dm> DmaSupportTx for DacContinuousTx<'d, Dm>
    where
        Dm: DriverMode,
    {
        type Channel = PeripheralTxChannel<SPI3<'d>>;

        fn tx(&mut self) -> &mut ChannelTx<Dm, PeripheralTxChannel<SPI3<'d>>> {
            &mut self.tx_channel
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.tx_chain
        }
    }
}

// Only specific pins can be used with each DAC peripheral, and of course
// these pins are different depending on which chip you are using; for this
// reason, we will type alias the pins for ease of use later in this module:
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        type Dac1Gpio<'d> = crate::peripherals::GPIO25<'d>;
        type Dac2Gpio<'d> = crate::peripherals::GPIO26<'d>;
    } else if #[cfg(esp32s2)] {
        type Dac1Gpio<'d> = crate::peripherals::GPIO17<'d>;
        type Dac2Gpio<'d> = crate::peripherals::GPIO18<'d>;
    }
}

/// Digital-to-Analog Converter (DAC) Channel
pub struct Dac<'d, T>
where
    T: Instance + 'd,
    T::Pin: AnalogPin + 'd,
{
    _inner: T,
    _lifetime: core::marker::PhantomData<&'d mut ()>,
}

impl<'d, T> Dac<'d, T>
where
    T: Instance + 'd,
    T::Pin: AnalogPin + 'd,
{
    /// Construct a new instance of [`Dac`].
    pub fn new(dac: T, pin: T::Pin) -> Self {
        // TODO: Revert on drop.
        pin.set_analog(crate::private::Internal);

        #[cfg(esp32s2)]
        crate::peripherals::SENS::regs()
            .sar_dac_ctrl1()
            .modify(|_, w| w.dac_clkgate_en().set_bit());

        T::enable_xpd();

        Self {
            _inner: dac,
            _lifetime: core::marker::PhantomData,
        }
    }

    /// Writes the given value.
    ///
    /// For each DAC channel, the output analog voltage can be calculated as
    /// follows: DACn_OUT = VDD3P3_RTC * PDACn_DAC/256
    pub fn write(&mut self, value: u8) {
        T::set_pad_source();
        T::write_byte(value);
    }
}

#[doc(hidden)]
pub trait Instance: crate::private::Sealed {
    const INDEX: usize;

    type Pin;

    fn enable_xpd() {
        crate::peripherals::RTC_IO::regs()
            .pad_dac(Self::INDEX)
            .modify(|_, w| w.dac_xpd_force().set_bit().xpd_dac().set_bit());
    }

    fn set_pad_source() {
        crate::peripherals::SENS::regs()
            .sar_dac_ctrl2()
            .modify(|_, w| w.dac_cw_en(Self::INDEX as u8).clear_bit());
    }

    fn write_byte(value: u8) {
        crate::peripherals::RTC_IO::regs()
            .pad_dac(Self::INDEX)
            .modify(|_, w| unsafe { w.dac().bits(value) });
    }
}

#[cfg(dac_dac1)]
impl<'d> Instance for crate::peripherals::DAC1<'d> {
    const INDEX: usize = 0;

    type Pin = Dac1Gpio<'d>;
}

#[cfg(dac_dac2)]
impl<'d> Instance for crate::peripherals::DAC2<'d> {
    const INDEX: usize = 1;

    type Pin = Dac2Gpio<'d>;
}
