#!/usr/bin/env python3

import time


from aal_msgs.msg import AdaptationState

import sys

from abc import ABC, abstractmethod

class AdaptationStrategy(ABC):
    def __init__(self,strategy_name):
        self.name = strategy_name + "_strategy"
        self.configuration_dict = {}
        
    def get_name(self):
        return self.name
    
    @abstractmethod
    def suggest_adaptation(adaptation_state):
        pass

    def parse_configurations(self, adaptation_state):
        for config_msg in adaptation_state.possible_configurations:
            string_repr = " ".join([str((param.name, param.value)) for param in config_msg.configuration_parameters] + config_msg.node_names + [str(adap_type) for adap_type in config_msg.adaptation_target_types])
            if string_repr not in self.configuration_dict:
                self.configuration_dict[string_repr] = config_msg
    
