#!/usr/bin/env python3
import numpy as np
import subprocess

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
        
    def suggest_adaptation(self, adaptation_state):
        print("\n\nstart\n\n")
        print(f"safety: {adaptation_state.safeties}")
        possible_configs = adaptation_state.possible_configurations
        model = '~/rebet_ws/prism-4.8.1-linux64-x86/prism-examples/simple/dice/dice.pm'
        props = '~/rebet_ws/prism-4.8.1-linux64-x86/prism-examples/simple/dice/dice.pctl'
        prop_index = '2,4'

        best_config = possible_configs[0]
        best_util = 0 #TODO: Maybe fix later

        for config in possible_configs:
            # Run PRSIM in subprocess
            completed_process = subprocess.run(
                [f'~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism {model} {props} -prop {prop_index}'],
                shell=True, capture_output=True, text=True)

            # Parse output
            prop_results = []
            output = completed_process.stdout

            for result_string in output.split("Result: ")[1:]:
                prop_results.append(float(result_string.split()[0]))

            print(prop_results)
            util = sum(prop_results) # TODO: rethink
            if util > best_util:
                best_config = config
                best_util = util

        print("\n\nend\n\n")
        return best_config
