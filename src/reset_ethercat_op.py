#!/usr/bin/env python3
import subprocess
import time
import sys

def check_ethercat_state():
    try:
        result = subprocess.run(['ethercat', 'slaves'], capture_output=True, text=True, check=True)
        lines = result.stdout.strip().split('\n')
        
        all_op = True
        has_error = False
        for line in lines:
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) >= 4:
                state = parts[2]
                error = '+' if '+' in parts[3:] else ('E' if 'E' in parts[3:] else '')
                
                if state != 'OP':
                    all_op = False
                if error == 'E':
                    has_error = True
        return all_op, has_error, lines
    except Exception as e:
        print(f"执行 ethercat 命令失败: {e}")
        return False, False, []

def reset_ethercat():
    print("检测到未就绪或错误状态，尝试发送 OP 状态请求以清除错误...")
    try:
        subprocess.run(['ethercat', 'states', 'OP'], check=True)
        time.sleep(2.0)  # 等待状态切换
    except Exception as e:
        print(f"尝试重置状态失败: {e}")

def main():
    print("检查 EtherCAT 从站状态...")
    max_retries = 5
    retries = 0
    
    while retries < max_retries:
        all_op, has_error, lines = check_ethercat_state()
        
        if all_op and not has_error:
            print("\n所有从站均已正常进入 OP 状态！")
            for line in lines:
                print("  " + line)
            sys.exit(0)
            
        print("\n当前状态：")
        for line in lines:
            print("  " + line)
            
        reset_ethercat()
        retries += 1
        
    print("\n无法将所有从站切换至 OP 状态，请检查硬件连接或配置。")
    sys.exit(1)

if __name__ == '__main__':
    main()
