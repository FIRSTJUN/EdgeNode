"""Give serialized safety inputs and RPCs time between expensive plan ticks."""
import time

def run_with_gap(timer,period_s,callback):
    started=time.monotonic_ns();timer.cancel()
    try:
        return callback()
    finally:
        elapsed=(time.monotonic_ns()-started)/1e9
        # Keep the nominal start-to-start period when computation fits.
        # An overrun must not leave a timer permanently ready ahead of RPCs.
        # This does not refresh any vehicle input or change a watchdog limit.
        timer.timer_period_ns=round(max(.02,period_s-elapsed)*1e9)
        timer.reset()
