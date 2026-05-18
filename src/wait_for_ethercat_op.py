#!/usr/bin/env python3
import subprocess
import time
import sys

def check_ethercat_state():
    try:
        result = subprocess.run(['ethercat', 'slaves'], capture_output=True, text=True, check=True)
        lines = result.stdout.strip().split('\n')
        
        all_op = True
        for line in lines:
            if not line.strip():
                continue
            
            parts = line.split()
            # 格式类似: 1  0:1  PREOP  E  EYOU_ServoModule_ECAT_V145
            if len(parts) >= 4:
                state = parts[2]
                error = '+' if '+' in parts[3:] else ('E' if 'E' in parts[3:] else '')
                
                if state != 'OP' or error == 'E':
                    all_op = False
                    return False, lines
        return all_op, lines
    except Exception as e:
        print(f"执行 ethercat 命令失败: {e}")
        return False, []

def main():
    print("等待所有 EtherCAT 从站进入 OP 状态...")
    try:
        while True:
            is_op, lines = check_ethercat_state()
            if is_op:
                print("\n所有从站均已进入 OP 状态！")
                for line in lines:
                    print("  " + line)
                sys.exit(0)
            else:
                print("\r当前状态尚未全部 OP，继续等待...", end='')
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n已取消等待。")
        sys.exit(1)

if __name__ == '__main__':
    main()
