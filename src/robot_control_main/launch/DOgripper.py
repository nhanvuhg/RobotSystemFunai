from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
import time

with CpxAp(ip_address="192.168.27.93") as myCPX:
    print("Doc path:", myCPX.docu_path)

    myIO = myCPX.modules[1]     # CPX-AP-8DI
    myIO.name = "cpxap8di"

    while True:
        myIO.reset_channel(0)
        myIO.set_channel(1)
        time.sleep(0.1)