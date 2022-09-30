#![no_std]
#![no_main]

use pico_probe as _;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [XIP_IRQ, CLOCKS_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use defmt::*;
    use embedded_hal::adc::OneShot;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use pico_probe::setup::*;
    use rp2040_hal::usb::UsbBus;
    use rp2040_monotonic::*;
    use usb_device::class_prelude::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {
        vcp: pico_probe::vcp::VCP,
        probe_usb: pico_probe::usb::ProbeUsb,
    }

    #[local]
    struct Local {
        dap_handler: DapHandler,
        led: LedPin,
        adc: AdcReader,
    }

    #[init(local = [
        usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
        delay: MaybeUninit<pico_probe::systick_delay::Delay> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (mono, led, probe_usb, dap_handler, vcp, adc, translator_power, target_power) =
            setup(cx.device, cx.core, cx.local.usb_bus, cx.local.delay);

        led_blinker::spawn().ok();

        (
            Shared { vcp , probe_usb},
            Local {
                dap_handler,
                led,
                adc,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, adc])]
    fn led_blinker(cx: led_blinker::Context) {
        cx.local.led.toggle().ok();
        let val = cx.local.adc.voltage();
        defmt::info!("Vtgt = {} mV", val);
        led_blinker::spawn_after(1000.millis()).ok();
    }

    #[task(binds = USBCTRL_IRQ, local = [dap_handler, resp_buf: [u8; 64] = [0; 64]], shared = [probe_usb, vcp])]
    fn on_usb(mut ctx: on_usb::Context) {
        let mut probe_usb = ctx.shared.probe_usb;
        let dap = ctx.local.dap_handler;
        let resp_buf = ctx.local.resp_buf;


        if let Some(request) = probe_usb.lock(|f| f.interrupt(true)) {
            use dap_rs::{dap::DapVersion, usb::Request};
            use pico_probe::usb::ProbeUsbRequest;

            match request {
                ProbeUsbRequest::DAP(dap_req) => match dap_req {
                    Request::DAP1Command((report, n)) => {
                        let len = dap.process_command(&report[..n], resp_buf, DapVersion::V1);

                        if len > 0 {
                            probe_usb.lock(|f| f.dap1_reply(&resp_buf[..len]));
                        }
                    }
                    Request::DAP2Command((report, n)) => {
                        let len = dap.process_command(&report[..n], resp_buf, DapVersion::V2);

                        if len > 0 {
                            probe_usb.lock(|f|f.dap2_reply(&resp_buf[..len]));
                        }
                    }
                    Request::Suspend => {
                        info!("Got USB suspend command");
                        dap.suspend();
                    }
                },
                ProbeUsbRequest::VCPPacket((buffer, n)) => {
                    ctx.shared.vcp.lock(|f| f.write(&buffer, n));
                }
            }
        }

    }

    #[idle(shared = [vcp, probe_usb])]
    fn idle(mut cx: idle::Context) -> ! {
       loop{
        let mut buffer = [0u8; 64];
        let n = cx.shared.vcp.lock(|f| f.read(&mut buffer));
        if n > 0 {
            cx.shared.probe_usb.lock(|f| f.vcp_reply(&buffer[..n]));
        }
       }
    }
}
