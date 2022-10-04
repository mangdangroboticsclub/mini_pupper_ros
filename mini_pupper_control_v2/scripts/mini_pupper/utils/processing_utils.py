from multiprocessing import Process
from typing import List, Tuple


def run_in_parallel(functions: List[Tuple]):
    processes: List[Process] = []
    for func in functions:
        process: Process = Process(target=func[0], args=func[1:] if len(func) > 1 else ())
        process.start()
        processes.append(process)
    for process in processes:
        process.join()
