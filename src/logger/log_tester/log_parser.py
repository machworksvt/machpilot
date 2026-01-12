import re
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import List

@dataclass
class ParsedLog:
    timestamp: datetime
    severity: str
    data: str
    raw_line: str

def verify_logs(log_output: str, expected_times: List[datetime], expected_types: List[str], expected_data: List[str], header_tolerance: timedelta,tolerance: timedelta):
    # Regex matches: Log{time: 2025-12-31 18:25:26.180 severity: LOG, data: ...}
    log_pattern = re.compile(
        r"Log\{time: (?P<time>[\d\- :.]+) severity: (?P<severity>\w+), data: (?P<data>.*)\}"
    )

    parsed_logs = []
    lines = log_output.strip().split('\n')
    
    # Parse all lines into ParsedLog
    for line in lines:
        match = log_pattern.search(line)
        if match:
            #extract time from line
            dt = datetime.strptime(match.group("time"), "%Y-%m-%d %H:%M:%S.%f")
            
            parsed_logs.append(ParsedLog(
                timestamp=dt,
                severity=match.group("severity"), 
                data=match.group("data"),
                raw_line=line
            ))

    print(f"--- Captured {len(parsed_logs)} logs vs {len(expected_types)} expected ---")

    all_passed = True
    
    print(f"{'EXPECTED TYPE':<15} | {'ACTUAL TYPE':<15} | {'EXPECTED DATA':<15} | {'ACTUAL DATA':<15} | {'DELTA (ms)':<10} | {'RESULT'}")
    print("-" * 91)

    for i, (log, exp_time, exp_type,exp_data) in enumerate(zip(parsed_logs, expected_times, expected_types,expected_data)):

        # Check Severity
        type_match = (log.severity == exp_type)
        
        # Check Time if within tolerance
        current_tolerance=header_tolerance if i==0 else tolerance
        diff = abs(log.timestamp - exp_time)
        time_match = diff <= current_tolerance
        
        # Convert diff to ms for display
        diff_ms = diff.total_seconds() * 1000

        #check data match
        data_match= (exp_data == log.data)
        
        single_pass=type_match and time_match and data_match
        status_str = "PASS" if single_pass else "FAIL"
        if not single_pass:
            all_passed = False
        
        print(f"{exp_type:<15} | {log.severity:<15} | {exp_data:<15} | {log.data:<15} | {diff_ms:<10.2f} | {status_str}")

        if not type_match:
            print(f"\tMismatch Type: Expected '{exp_type}' != Got '{log.severity}'")
        if not time_match:
            print(f"\tMismatch Time: Diff {diff_ms:.2f}ms > {current_tolerance.total_seconds()*1000}ms")
        if not data_match:
            print(f"\tMismatch Data: Expected '{exp_data}' != Got '{log.data}'")

    if len(parsed_logs) != len(expected_types):
        print(f"\tCount mismatch: Parsed {len(parsed_logs)} but expected {len(expected_types)}.")
        all_passed = False

    return all_passed


if __name__=="__main__":
    sample_output = """
    Log{time: 2025-12-31 18:25:26.180 severity: LOG, data: LoggerStartup}
    Log{time: 2025-12-31 18:25:28.103 severity: LOG, data: Heartbeat}
    Log{time: 2025-12-31 18:25:28.204 severity: WARN, data: Heartbeat}
    Log{time: 2025-12-31 18:25:28.405 severity: ERROR, data: Heartbeat}
    Log{time: 2025-12-31 18:25:28.705 severity: LOG, data: Heartbeat}
    Log{time: 2025-12-31 18:25:29.106 severity: WARN, data: Heartbeat}
    """


    expected_types_refactored = ["LOG", "LOG", "WARN", "ERROR", "LOG", "WARN"]

    mock_times = [
        datetime(2025, 12, 31, 18, 25, 26, 180000),# Startup
        datetime(2025, 12, 31, 18, 25, 28, 103000),# Log 1
        datetime(2025, 12, 31, 18, 25, 28, 204000),# Warn 1
        datetime(2025, 12, 31, 18, 25, 28, 405000),# Error
        datetime(2025, 12, 31, 18, 25, 28, 705000),# Log 2
        datetime(2025, 12, 31, 18, 25, 29, 106000),# Warn 2
    ]

    expected_data=["LoggerStartup"]+5*["Heartbeat"]

    verify_logs(
        sample_output, 
        mock_times, 
        expected_types_refactored,
        expected_data,
        timedelta(milliseconds=500),
        timedelta(milliseconds=50),
    )