import subprocess

class CommandExecutor:
    @staticmethod
    def execute(cmd_args):
        """执行命令并返回结果"""
        try:
            result = subprocess.run(cmd_args, capture_output=True, text=True)
            return result
        except Exception as e:
            print(f"执行错误: {e}")
            return None

    @staticmethod
    def print_result(cmd_args, result):
        """打印命令执行结果"""
        print(f"\n执行: {' '.join(cmd_args)}")
        print(f"返回码: {result.returncode}")
        if result.stdout: print(f"输出:\n{result.stdout}")
        if result.stderr: print(f"错误:\n{result.stderr}")