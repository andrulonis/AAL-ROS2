#!/usr/bin/env python3
import numpy as np
import subprocess

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism')
        
    def suggest_adaptation(self, adaptation_state):
        print("\n\nstart\n\n")
        print(f"utility: {adaptation_state.current_utility}")
        print(f"safety: {adaptation_state.safeties}")
        print(f"power: {adaptation_state.powers}")
        print(f"move: {adaptation_state.moves}")
        possible_configs = adaptation_state.possible_configurations
        prism_dir = "~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism"
        model = '~/rebet_ws/src/aal/aal/aal/adaptation_strategies/models/model.pm' 
        props = '~/rebet_ws/src/aal/aal/aal/adaptation_strategies/models/base_mission.pctl' 

        best_config = possible_configs[0]
        best_util = float('-inf')

        abs_path = '/home/andrulonis/rebet_ws/src/aal/aal/aal/adaptation_strategies/models' #TODO: fix the pathing

        with open(f'{abs_path}/base_mission.pm','r') as base_model_file:
            base_model = base_model_file.read()

        for config in possible_configs:
            speed = config.configuration_parameters[0].value.double_array_value[0]

            with open(f'{abs_path}/model.pm','w') as model_file:
                model_file.write(base_model)
                model_file.write(f'const double safety = {adaptation_state.safeties[0]};')
                model_file.write(f'const double power = {adaptation_state.powers[0]};')
                model_file.write(f'const double move = {adaptation_state.moves[0]};')
                model_file.write(f'const double speed = {speed};')
                
            # Run PRSIM in subprocess
            completed_process = subprocess.run(
                [f'~/rebet_ws/prism-4.8.1-linux64-x86/bin/prism {model} {props}'],
                shell=True, capture_output=True, text=True)

            # Parse output
            prop_results = []
            output = completed_process.stdout

            for result_string in output.split("Result: ")[1:]:
                prop_results.append(float(result_string.split()[0]))

            print(prop_results)
            util = speed # TODO: rethink
            if util > best_util:
                best_config = config
                best_util = util

        print("\n\nend\n\n")
        return best_config
