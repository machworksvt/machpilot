from dataclasses import dataclass
import datetime


@dataclass
class Log:
    expected_sleep_time: float
    time_sent: datetime
    severity: str
    source: str
    data: str