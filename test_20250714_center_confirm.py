from lunarprinter_ctrler import printer_ctrl

if __name__ == '__main__':
    ctrl_params_file = 'ctrl_params.xml'
    printer = printer_ctrl(ctrl_params_file)
    printer.center_confirm()