import re
import subprocess
import time
import signal
import os
import datetime
import log_parser
import random

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


file_writer_process = create_ros2_process(
    "ros2 run log_file_manager log_file_writer_node --ros-args -p log_file:=\"src/logger/log_tester/test_log\""
)
log_printer_process = create_ros2_process(
    "ros2 run printer_logger_node printer_logger_node"
)
log_creater_process = create_ros2_process(
    "ros2 run example_logging_node2 ExampleLoggingNode2"
)
logger_start_up_time=datetime.datetime.now()

# Give ROS2 nodes a moment to discover each other
time.sleep(4)


#====================================Create Logs====================================
log_counts=random.randint(15,20)

goal_times=[random.uniform(0,2) for _ in range(log_counts)]
goal_times.sort()
log_types= [random.choice(["LOG","WARN","ERROR"]) for _ in range(log_counts)]
log_times=[]


print("Sending logs...")
start_log_time=datetime.datetime.now()
for offset, log_type in zip(goal_times, log_types):

    #wait until offset time since start_log_time
    wait_time=(start_log_time+datetime.timedelta(seconds=offset))-datetime.datetime.now()
    time.sleep(wait_time.microseconds/1_000_000)
    log_times.append(datetime.datetime.now())
    
    # Check if the process is already dead
    if log_creater_process.poll() is not None:
        print("Process died unexpectedly")
        print("Error Output:", log_creater_process.stderr.read()) 
        exit(1)
    else:
        #write command
        log_creater_process.stdin.write(log_type + "\n")
        log_creater_process.stdin.flush()
# Allow some time for logs to process before shutting down
time.sleep(1)

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

printer_passed=log_parser.verify_logs(
    stdout_data,
    [logger_start_up_time]+log_times,
    ["LOG"]+log_types,
    ["LoggerStartup"]+log_counts*["Heartbeat"],
    datetime.timedelta(seconds=1),
    datetime.timedelta(milliseconds=50),
)

if not printer_passed:
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
    print("coulnd not parse file_writer_process output")
    exit(1)
file=match.group("file")

print(f"{file=}")

file_logs=subprocess.run(get_ros2_command(f"install/log_file_manager/lib/log_file_manager/log_file_reader {file}"),stdout=subprocess.PIPE,text=True,shell=True)
os.remove(file)
stdout_data = file_logs.stdout

file_passed=log_parser.verify_logs(
    stdout_data,
    [logger_start_up_time]+log_times,
    ["LOG"]+log_types,
    ["LoggerStartup"]+log_counts*["Heartbeat"],
    datetime.timedelta(seconds=1),
    datetime.timedelta(milliseconds=50),
)

if not file_passed:
    exit(1)


