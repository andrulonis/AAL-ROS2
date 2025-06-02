#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

# Get path to PRISM binary
prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"

class PrismMDPStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('mdp')

    # Check if all necessary files are provided
    def check_files_present(self):
        if not os.path.exists(self.model_dir):
            print(f"Path {self.model_dir} does not exist, please provide a correct path.")
            return False
        
        if not os.path.isfile(f"{self.model_dir}/base_model.pm"):
            print(f"File base_model.pm is missing from the given directory.")
            return False
            
        if not os.path.isfile(f"{self.model_dir}/property.pctl"):
            print(f"File property.pctl is missing from the given directory.")
            return False
        return True
    
    # Write qr-metrics and context values to the model (all assumed to be given as doubles) and state variables (config) as initial values (assumed to be given as ints)
    def write_qr_and_context_and_config_to_model(self, qrs, context, config):
        with open(f'{self.model_dir}/base_model.pm','r') as base_model_file:
            self.final_model = base_model_file.read()
        for qr in qrs:
            self.final_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'
        for kv in context:
            self.final_model += f'\nconst double {kv.key.lower()} = {kv.value};'
        for kv in config:
            self.final_model += f'\nconst {kv.key.lower()}_init = {kv.value};'
 
    def suggest_adaptation(self, adaptation_state):
        # Get path to the model, property from the user (from the behaviour tree)
        self.model_dir = adaptation_state.model_dir

        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]

        if not self.check_files_present():
            return chosen_config

        with open(f'{self.model_dir}/property.pctl') as property_file:
            prop = property_file.readline()

        qrs = adaptation_state.qrs
        context = adaptation_state.context
        config = adaptation_state.config

        self.write_qr_and_context_and_config_to_model(qrs, context, config)

        with open(f'{self.model_dir}/final_model.pm','w') as model_file:
            model_file.write(self.final_model)
        
        # Generate optimal strategy optimising over specified property
        completed_process = subprocess.run(
            [f'{prism_bin} {self.model_dir}/final_model.pm -pf \'{prop}\' -exportstrat stdout:reach=false'],
            shell=True, capture_output=True, text=True)
        
        # Parse output to obtain strategy
        output = completed_process.stdout
        strategy = {}
        strategy_string = output.split("Exporting strategy as actions below:\n")[1].split("\n---")[0]

        if strategy_string == [output]:
            print("PRISM did not give expected output. There may be a syntax error in the model, or some parameters did not get written properly.")
            return chosen_config

        for line in strategy_string.splitlines():
            (state, action) = line.split('=')
            # Have state as an array of values
            if action:
                config_index = int(action.split("config")[1])   
                strategy[state] = config_index

        current_state = '('
        for param in adaptation_state.config:
            current_state += param.value + ','
        current_state = current_state[:-1] + ')'

        if current_state in strategy:
            choice = strategy[current_state]
            chosen_config = possible_configs[choice]

        return chosen_config
    