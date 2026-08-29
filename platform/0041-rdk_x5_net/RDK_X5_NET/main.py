import sys
from py_bash import CommandExecutor
from py_bash import InteractiveCLI

def main():
    if len(sys.argv) == 1:
        cli = InteractiveCLI()
        cli.start()
    else:
        if result := CommandExecutor.execute(sys.argv[1:]):
            # 打印解析的参数（原始需求）
            parsed_args = ', '.join(f'"{arg}"' for arg in sys.argv[1:])
            print(f"解析的参数: {parsed_args}")
            CommandExecutor.print_result(sys.argv[1:], result)

if __name__ == "__main__":
    main()
