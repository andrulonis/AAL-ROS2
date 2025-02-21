#!/usr/bin/env python3
import numpy as np
import subprocess

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
        
    def suggest_adaptation(self, adaptation_state):
        possible_configs = adaptation_state.possible_configurations

        subprocess.run(['~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism ~/rebet_ws/prism-4.8.1-linux64-x86/prism-examples/simple/dice/dice.pm'], shell=True)

        print("\n\n\n\n\n DUPA \n\n\n\n")

        chosen_config = np.random.choice(possible_configs)

        return chosen_config
