use embedded_hal::serial::Write;
use heapless::spsc::Consumer;
use heapless::spsc::Producer;
use heapless::spsc::Queue;
use rp2040_hal::{
    clocks::PeripheralClock,
    gpio::{
        bank0::{Gpio24, Gpio25},
        Function, Pin, Uart,
    },
    multicore::{Core, Stack},
    pac::{UART1},
    uart::{Enabled, UartConfig, UartPeripheral},
    Clock,
};
use rp2040_monotonic::fugit::HertzU32;

use usbd_serial::{ParityType, StopBits};

type UartVCP =
    UartPeripheral<Enabled, UART1, (Pin<Gpio24, Function<Uart>>, Pin<Gpio25, Function<Uart>>)>;

#[cfg(feature = "usb-hs")]
pub const VCP_PACKET_SIZE: usize = 512;
#[cfg(not(feature = "usb-hs"))]
pub const VCP_PACKET_SIZE: usize = 64;

/// UART configuration struct
#[derive(PartialEq, Eq, Clone, Copy)]
pub struct VcpConfig {
    stop_bits: StopBits,
    data_bits: u8,
    parity_type: ParityType,
    data_rate: u32,
}

impl Default for VcpConfig {
    fn default() -> Self {
        VcpConfig {
            stop_bits: StopBits::One,
            data_bits: 8,
            parity_type: ParityType::None,
            data_rate: 9600,
        }
    }
}
static mut VCP_TX_BUFFER: Queue<u8, VCP_PACKET_SIZE> = Queue::new();
static mut VCP_RX_BUFFER: Queue<u8, VCP_PACKET_SIZE> = Queue::new();
static mut VCP_CONFIG_QUEUE: Queue<UartConfig, 2> = Queue::new();
pub struct VCP {
    tx_prod: Producer<'static, u8, VCP_PACKET_SIZE>,
    rx_cons: Consumer<'static, u8, VCP_PACKET_SIZE>,
    config_prod: Producer<'static, UartConfig, 2>,
}

impl VCP {
    pub fn new(uart: UartVCP, clock: &PeripheralClock, core1: &mut Core) -> Self {
        let (vcp_tx_producer, vcp_tx_consumer) = unsafe { VCP_TX_BUFFER.split() };
        let (vcp_rx_producer, vcp_rx_consumer) = unsafe { VCP_RX_BUFFER.split() };
        let (vcp_config_producer, vcp_config_consumer) = unsafe { VCP_CONFIG_QUEUE.split() };

        let pclk = clock.freq().to_Hz();
        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || -> ! {
            core1_task(uart, vcp_tx_consumer, vcp_rx_producer, vcp_config_consumer, pclk);
        });
        VCP {
            tx_prod: vcp_tx_producer,
            rx_cons: vcp_rx_consumer,
            config_prod: vcp_config_producer,
        }
    }

    /// Call with the system clock speeds to configure peripherals that require timing information.
    ///
    /// Currently this only configures the pins & DMA RX
    pub fn setup(&mut self) {
        self.config_prod.enqueue(UartConfig::default()).map_err(|_| rp2040_hal::uart::Error::BadArgument).unwrap();
    }

    /// Start the VCP function.
    ///
    /// This enables both TX & RX.
    pub fn start(&mut self) {
        //self.last_idx_rx = 0;
        //self.last_idx_tx = 0;
    }

    /// Disable UART.
    pub fn stop(&self) {
        // modify_reg!(
        //     usart,
        //     self.uart,
        //     CR1,
        //     RE: Disabled,
        //     TE: Disabled,
        //     UE: Disabled
        // );
    }

    /// Fetch current number of bytes available.
    ///
    /// Subsequent calls to read() may return a different amount of data.
    pub fn rx_bytes_available(&mut self) -> usize {
        self.rx_cons.len()
    }

    /// Read new UART data.
    ///
    /// Returns number of bytes written to buffer.
    ///
    /// Reads at most rx.len() new bytes, which may be less than what was received.
    /// Remaining data will be read on the next call, so long as the internal buffer
    /// doesn't overflow, which is not detected.
    pub fn read(&mut self, rx: &mut [u8]) -> usize {
        let len = self.rx_bytes_available();
        // dequeue byte per byte into rx until either rx is full or we run out of bytes
        for i in 0..len {
            if i >= rx.len() {
                return i;
            }
            rx[i] = self.rx_cons.dequeue().unwrap();
        }
        len
    }

    /// Setup the USART line config.
    ///
    /// This should be done between a `stop()` and a `start` call since
    /// configuring this requires the UE bit to be `0b0`.
    pub fn set_config(mut self, config: UartConfig) -> Result<(), rp2040_hal::uart::Error> {
        // translate ACM config into USART config
        // let mut config = UartConfig::default();
        // config.baudrate = HertzU32::from_raw(coding.data_rate);
        // config.data_bits = match coding.data_bits {
        //     5 => rp2040_hal::uart::DataBits::Five,
        //     6 => rp2040_hal::uart::DataBits::Six,
        //     7 => rp2040_hal::uart::DataBits::Seven,
        //     8 => rp2040_hal::uart::DataBits::Eight,
        //     _ => panic!("Invalid data bits"), //    rp2040_hal::uart::DataBits::Eight
        // };
        // config.stop_bits = match coding.stop_bits {
        //     StopBits::One => rp2040_hal::uart::StopBits::One,
        //     StopBits::Two => rp2040_hal::uart::StopBits::Two,
        //     StopBits::OnePointFive => {
        //         panic!("Invalid stop bits");
        //         rp2040_hal::uart::StopBits::One
        //     }
        // };
        // config.parity = match coding.parity_type {
        //     ParityType::None => None,
        //     ParityType::Odd => Some(rp2040_hal::uart::Parity::Odd),
        //     ParityType::Event => Some(rp2040_hal::uart::Parity::Even),
        //     ParityType::Mark => None,  // unsupported?
        //     ParityType::Space => None, // unsupported?
        // };

        self.config_prod
            .enqueue(config)
            .map_err(|_| rp2040_hal::uart::Error::BadArgument)
    }

    /// Check state of TX Dma transfer
    pub fn is_tx_idle(&self) -> bool {
        //self.dma.usart2_tx_ndtr() == 0
        true
    }
    /// Start DMA transfer from buffer to TX Shift register.
    pub fn write(&mut self, tx: &[u8], len: usize) {
        // enqueue tx into the tx buffer
        for word in tx.iter().take(len) {
            self.tx_prod.enqueue(*word).unwrap();
        }
    }
}

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(
    mut uart: UartVCP,
    mut tx_cons: Consumer<'static, u8, VCP_PACKET_SIZE>,
    mut rx_prod: Producer<'static, u8, VCP_PACKET_SIZE>,
    mut config_cons: Consumer<'static, UartConfig, 2>,
    pclk: u32,
) -> ! {
    loop {
        if let Some(config) = config_cons.dequeue() {
            uart = uart.disable().enable(config, HertzU32::from_raw(pclk)).unwrap();
        }
        // if we have tx data to consume and send trough uart, dequeue one byte and send it
        if tx_cons.len() > 0 {
            let byte = tx_cons.dequeue().unwrap();
            uart.write(byte).unwrap();
        }

        // if we have RX data to produce, read one byte from uart and enqueue it
        if rx_prod.ready() {
            // use read raw into buffer of 16 bytes, ane enqueue as much as we read
            let mut buffer = [0u8; 16];
            let len = uart.read_raw(&mut buffer).unwrap_or(0);
            for item in buffer.iter().take(len) {
                rx_prod.enqueue(*item).unwrap();
            }
        }
    }
}
