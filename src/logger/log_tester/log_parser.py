import re
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import List

from log_type import Log

@dataclass
class ParsedLog:
    timestamp: datetime
    severity: str
    source: str
    data: str
    raw_line: str

def verify_logs(log_output: str, expected_logs: List[Log], header_tolerance: timedelta,tolerance: timedelta):
    
    log_pattern = re.compile(
        r"Log\{time: (?P<time>[\d\- :.]+), source: (?P<source>\w+), severity: (?P<severity>\w+), data: (?P<data>.*)\}"
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
                source=match.group("source"),
                data=match.group("data"),
                raw_line=line
            ))
        else:
            print(f"failed to parse line \"{line}\"")
            return False
        
    all_passed = True
    
    print(f"{'EXPECTED SEVERITY':<17} | {'ACTUAL SEVERITY':<17} | {'EXPECTED SOURCE':<15} | {'ACTUAL SOURCE':<15} | {'EXPECTED DATA':<15} | {'ACTUAL DATA':<15} | {'DELTA (ms)':<10} | {'RESULT'}")
    print("-" * 131)

    for i, (parsed_log,expected_log) in enumerate(zip(parsed_logs,expected_logs,strict=False)):

        # Check Severity
        severity_match = (parsed_log.severity == expected_log.severity)
        
        # Check Source
        source_match = (parsed_log.source == expected_log.source)


        # Check Time if within tolerance
        current_tolerance=header_tolerance if i==0 else tolerance
        diff = abs(parsed_log.timestamp - expected_log.time_sent)
        time_match = diff <= current_tolerance
        
        # Convert diff to ms for display
        diff_ms = diff.total_seconds() * 1000

        #check data match
        data_match= (expected_log.data == parsed_log.data)
        
        single_pass=severity_match and time_match and data_match and source_match
        status_str = "PASS" if single_pass else "FAIL"
        if not single_pass:
            all_passed = False
        
        print(f"{expected_log.severity:17} | {parsed_log.severity:<17} | {expected_log.source:<15} | {parsed_log.source:<15} | {expected_log.data:<15} | {parsed_log.data:<15} | {diff_ms:<10.2f} | {status_str}")

        if not severity_match:
            print(f"\tMismatch Type: Expected '{expected_log.severity}' != Got '{parsed_log.severity}'")
        if not time_match:
            print(f"\tMismatch Time: Diff {diff_ms:.2f}ms > {current_tolerance.total_seconds()*1000}ms")
        if not data_match:
            print(f"\tMismatch Data: Expected '{expected_log.data}' != Got '{parsed_log.data}'")

    if len(parsed_logs) != len(expected_logs):
        print(f"\tCount mismatch: Parsed {len(parsed_logs)} but expected {len(expected_logs)}.")
        all_passed = False

    return all_passed