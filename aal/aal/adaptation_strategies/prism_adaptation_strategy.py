#!/usr/bin/env python3
import numpy as np
import subprocess

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
        
    def suggest_adaptation(self, adaptation_state):
        possible_configs = adaptation_state.possible_configurations
        
        # Run PRSIM in subprocess
        completed_process = subprocess.run(
            ['~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism -dir ~/rebet_ws/prism-4.8.1-linux64-x86/prism-examples/simple/dice dice.pm dice.pctl -prop 2'],
            shell=True, capture_output=True, text=True
            )

        # Parse output
        output = completed_process.stdout
        i = output.find("Result:")
        prop_result = float(output[i:].split()[1])

        chosen_config = np.random.choice(possible_configs)
        return chosen_config
