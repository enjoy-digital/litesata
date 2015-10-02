import sys
from tools import *
from bist import *
from litescope.software.driver.logic_analyzer import LiteScopeLogicAnalyzerDriver


def main(wb):
    logic_analyzer = LiteScopeLogicAnalyzerDriver(wb.regs, "logic_analyzer", debug=True)
    identify = LiteSATABISTIdentifyDriver(wb.regs, "sata_bist")
    generator = LiteSATABISTGeneratorDriver(wb.regs, "sata_bist")
    checker = LiteSATABISTCheckerDriver(wb.regs, "sata_bist")
    wb.open()
    regs = wb.regs
    # # #
    trig = "now"
    if len(sys.argv) < 3:
        print("No trigger condition, triggering immediately!")
    else:
        trig = sys.argv[2]

    conditions = {}
    conditions["now"] = {}
    conditions["id_cmd"] = {
        "sata_command_tx_sink_stb": 1,
        "sata_command_tx_sink_payload_identify": 1,
    }
    conditions["id_resp"] = {
        "source_source_payload_data": primitives["X_RDY"],
    }
    conditions["wr_cmd"] = {
        "sata_command_tx_sink_stb": 1,
        "sata_command_tx_sink_payload_write": 1,
    }
    conditions["wr_resp"] = {
        "sata_command_rx_source_stb": 1,
        "sata_command_rx_source_payload_write": 1,
    }
    conditions["rd_cmd"] = {
        "sata_command_tx_sink_stb": 1,
        "sata_command_tx_sink_payload_read": 1,
    }
    conditions["rd_resp"] = {
        "sata_command_rx_source_stb": 1,
        "sata_command_rx_source_payload_read": 1,
    }

    logic_analyzer.configure_term(port=0, cond=conditions[trig])
    logic_analyzer.configure_sum("term")

    # Run Logic Analyzer
    logic_analyzer.run(offset=64, length=1024)

    #identify.run(blocking=False)
    generator.run(0, 2, 1, 0, blocking=False)
    #checker.run(0, 2, 1, 0, blocking=False)

    while not logic_analyzer.done():
        pass

    logic_analyzer.upload()
    logic_analyzer.save("dump.vcd")
    # # #
    wb.close()

    f = open("dump_link.txt", "w")
    data = link_trace(logic_analyzer,
        tx_data_name="sink_sink_payload_data",
        rx_data_name="source_source_payload_data"
    )
    f.write(data)
    f.close()
