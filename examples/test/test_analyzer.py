from litex import RemoteClient
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver
from litescope.software.dump import *

from test_bist import *

wb = RemoteClient()
wb.open()

# # #

analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)

cond = {"bistsocdevel_command_tx_sink_valid" : 1,
        "bistsocdevel_command_tx_sink_ready" : 1}
#analyzer.configure_trigger(cond=cond)
analyzer.configure_trigger(cond={})
analyzer.run(offset=32, length=64)

#generator = LiteSATABISTGeneratorDriver(wb.regs, wb.constants, "sata_bist")
#generator.run(0, 1, 1, 0, True)

analyzer.wait_done()
analyzer.upload()
analyzer.save("dump.vcd")

# analyze link layer

primitives = {
    "ALIGN":  0x7B4A4ABC,
    "CONT":   0X9999AA7C,
    "SYNC":   0xB5B5957C,
    "R_RDY":  0x4A4A957C,
    "R_OK":   0x3535B57C,
    "R_ERR":  0x5656B57C,
    "R_IP":   0X5555B57C,
    "X_RDY":  0x5757B57C,
    "CONT":   0x9999AA7C,
    "WTRM":   0x5858B57C,
    "SOF":    0x3737B57C,
    "EOF":    0xD5D5B57C,
    "HOLD":   0xD5D5AA7C,
    "HOLDA":  0X9595AA7C
}

def decode_primitive(dword):
    for k, v in primitives.items():
        if dword == v:
            return k
    return ""

def analyze_link_layer(analyzer, tx_data_name, rx_data_name):
    r = ""
    dump = Dump()
    dump.add_from_layout(analyzer.layout, analyzer.data)

    for variable in dump.variables:
        if variable.name == tx_data_name:
            tx_data = variable.values
        if variable.name == rx_data_name:
            rx_data = variable.values

    for i in range(len(tx_data)):
        tx = "{:08x} ".format(tx_data[i])
        tx += decode_primitive(tx_data[i])
        tx += " "*(16-len(tx))

        rx = "{:08x} ".format(rx_data[i])
        rx += decode_primitive(rx_data[i])
        rx += " "*(16-len(rx))

        r += tx + rx + "\n"

    return r


f = open("link_layer.txt", "w")
data = analyze_link_layer(analyzer,
        tx_data_name="bistsocdevel_datapath_sink_sink_payload_data",
        rx_data_name="bistsocdevel_datapath_source_source_payload_data"
        )
f.write(data)
f.close()

# # #

wb.close()
