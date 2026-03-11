import re
import subprocess
import time
import signal
import os
import datetime
import log_parser
import random
import os

from log_type import Log


# Helper to construct the full shell command
def get_ros2_command(cmd_args):
    setup_path = "install/setup.bash"
    # We chain commands with '&&' so the environment from 'source' persists for the 'ros2' command
    # We use "bash -c" to explicitly run these commands in a bash shell since Popen only likes to deal with a single exicutable
    return f"bash -c 'source {setup_path} && {cmd_args}'"

def create_ros2_process(command):
    process = subprocess.Popen(
        get_ros2_command(command),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        shell=True,
        preexec_fn=os.setsid # logic to allow killing the whole process group later
    )
    return process

log_directory =os.environ.get('RUNNER_TEMP',default="src/logger/log_tester/test_log")

file_writer_process = create_ros2_process(
    f"ros2 run log_file_manager log_file_writer_node --ros-args -p log_file:=\"{log_directory}\""
)
log_printer_process = create_ros2_process(
    "ros2 run printer_logger_node printer_logger_node"
)
log_creater_process = create_ros2_process(
    "ros2 run log_tester_node log_tester_node"
)
logger_start_up_time=datetime.datetime.now()

start_up_log=Log(0,logger_start_up_time,"LOG","Logger","LoggerStartup")

# Give ROS2 nodes a moment to discover each other
print("letting nodes start up")
time.sleep(4)


#====================================Create Logs====================================
print("Sending logs...")
log_counts=random.randint(15,20)


logs=[]

for _ in range(log_counts):
    goal_times=random.uniform(0,2)
    severity=random.choice(["LOG","WARN","ERROR"])
    source=random.choice(["LoggerTester0","LoggerTester1","LoggerTester2"])
    data="Heartbeat"

    logs.append(Log(goal_times,None,severity,source,data))

logs.sort(key=lambda l:l.expected_sleep_time)


start_log_time=datetime.datetime.now()
for log in logs:

    #wait until offset time since start_log_time
    wait_time=(start_log_time+datetime.timedelta(seconds=log.expected_sleep_time))-datetime.datetime.now()
    time.sleep(wait_time.microseconds/1_000_000)
    log.time_sent=datetime.datetime.now()
    
    # Check if the process is already dead
    if log_creater_process.poll() is not None:
        print("Process died unexpectedly")
        print("Error Output:", log_creater_process.stderr.read()) 
        exit(1)
    else:
        #write command
        log_creater_process.stdin.write(f"{log.severity} {log.source}\n")
        log_creater_process.stdin.flush()
logs.insert(0,start_up_log)
print("giving time for nodes to prossess logs before shutdown")
# Allow some time for logs to process before shutting down
time.sleep(0.1)

# Terminate processes before reading to prevent hanging
# We kill the printer process so its stdout pipe closes, allowing .read() to finish.
os.killpg(os.getpgid(log_printer_process.pid), signal.SIGTERM)
os.killpg(os.getpgid(file_writer_process.pid), signal.SIGTERM)
os.killpg(os.getpgid(log_creater_process.pid), signal.SIGTERM)

#====================================Check Printer====================================
print("====================================Check Printer====================================")
stdout_data, stderr_data = log_printer_process.communicate()

print("--- Printer Errors (if any) ---")
print(stderr_data)

print("results for direct printing")
printer_passed=log_parser.verify_logs(
    stdout_data,
    logs,
    datetime.timedelta(seconds=1),
    datetime.timedelta(milliseconds=50),
)

if not printer_passed:
    print("direct printing failed")
    exit(1)

#====================================Check File====================================
print("====================================Check File====================================")

stdout_data, stderr_data = file_writer_process.communicate()

print("--- Printer Output ---")
print(stdout_data)

print("--- Printer Errors (if any) ---")
print(stderr_data)

log_pattern = re.compile(
    r"Log file being written to \"(?P<file>.*)\""
)

match = log_pattern.search(stdout_data)

if not match:
    print("could not parse file_writer_process output")
    exit(1)
file=match.group("file")

print(f"{file=}")



file_logs=subprocess.run(get_ros2_command(f"install/log_file_manager/lib/log_file_manager/log_file_reader {file}"),stdout=subprocess.PIPE,text=True,shell=True)
os.remove(file)
stdout_data = file_logs.stdout

print("results for file printing")
file_passed=log_parser.verify_logs(
    stdout_data,
    logs,
    datetime.timedelta(seconds=1),
    datetime.timedelta(milliseconds=50),
)

if not file_passed:
    print("file printing failed")
    exit(1)



print("test_passed")