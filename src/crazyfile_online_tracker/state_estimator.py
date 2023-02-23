#!/usr/bin/env python3
from abc import ABC, abstractmethod


class StateEstimator(ABC):
    def __init__(self):
        self.state_pub = None
        self.state = None

    @abstractmethod
    def state_pub_init(self):
        pass

    @abstractmethod
    def publish_state(self):
        pass
