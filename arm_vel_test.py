from lunarprinter_ctrler import arm_status

if __name__ == '__main__':
    ip = '192.168.1.101'
    port = 6001
    arm = arm_status(ip)

    arm.open_boardcast()
    data = arm.get_arm_status()
    print(data)