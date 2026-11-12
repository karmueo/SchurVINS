#!/usr/bin/env python3
"""Run an isolated ROS2 process with a per-run parameter file and resource samples."""
import os
from pathlib import Path
import subprocess
import threading
import numpy as np
import psutil
import yaml


def log_cpu_usage_thread(trace_dir, stop_event):
    cpu, mem = [], []
    while not stop_event.wait(0.5):
        cpu.append(psutil.cpu_percent(percpu=True))
        mem.append([psutil.virtual_memory().percent, psutil.swap_memory().percent])
    np.savetxt(Path(trace_dir)/'log_cpu_usage.txt', np.asarray(cpu), fmt='%.2e')
    np.savetxt(Path(trace_dir)/'log_memory_usage.txt', np.asarray(mem), fmt='%.2e')


class RosNode:
    def __init__(self, package, executable):
        self._package, self._executable = package, executable
        self._parameters, self._flags = {}, []

    def add_parameters(self, namespace, parameters):
        for key, value in list(parameters.items()):
            if key == 'flags':
                continue
            name = (namespace + key).strip('/').replace('/', '.')
            if isinstance(value, dict):
                self.add_parameters(name + '.', value)
            else:
                self._parameters[name] = value

    def add_flags(self, flags):
        self._flags = [f'--{key}={value}' for key, value in list(flags.items())]

    def run(self, parameter_dictionary, namespace='', log_cpu_usage=True,
            log_cpu_usage_folder='/tmp'):
        self._parameters, self._flags = {}, []
        self.add_parameters(namespace, parameter_dictionary)
        self.add_flags(parameter_dictionary.get('flags', {}))
        output = Path(log_cpu_usage_folder)
        output.mkdir(parents=True, exist_ok=True)
        config = output / 'params_ros.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': self._parameters}}))
        stop = threading.Event()
        thread = threading.Thread(target=log_cpu_usage_thread, args=(output, stop))
        if log_cpu_usage:
            thread.start()
        try:
            subprocess.run(['ros2', 'run', self._package, self._executable,
                            *self._flags, '--ros-args', '--params-file', str(config)], check=True)
        finally:
            stop.set()
            if log_cpu_usage:
                thread.join()
