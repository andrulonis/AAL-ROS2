#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismMDPStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('mdp')
 
    def suggest_adaptation(self, adaptation_state):
        # Get path to the model, property from the user (from the behaviour tree)
        model_dir = adaptation_state.model_dir

        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]

        if not os.path.exists(model_dir):
            print(f"Path {model_dir} does not exist, please provide a correct path.")
            return chosen_config
        
        if not os.path.isfile(f"{model_dir}/base_model.pm"):
            print(f"File base_model.pm is missing from the given directory.")
            return chosen_config
            
        if not os.path.isfile(f"{model_dir}/property.pctl"):
            print(f"File property.pctl is missing from the given directory.")
            return chosen_config
            
        if not os.path.isfile(f"{model_dir}/required_vars.txt"):
            print(f"File required_vars.txt is missing from the given directory.")
            return chosen_config
        
        # Get path to PRISM program
        prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"

        with open(f'{model_dir}/property.pctl') as property_file:
            prop = property_file.readline()
        
        # Initially set chosen configuration to the first one provided

        # Write metrics and context to model to obtain up-to-date model

        # Check if all required parametrised PRISM variables are provided (sometimes not the case in first few iterations),
        # and in case of strings, store possible values of the string in a dictionairy
        str_vars = {}
        keys = [kv.key.lower() for kv in adaptation_state.context] + \
               [qr.qr_name.lower() for qr in adaptation_state.qrs] + \
               [param.name for param in possible_configs[0].configuration_parameters]
        
        # TODO: if no required vars then ignore this step
        with open(f'{model_dir}/required_vars.txt', 'r') as required_vars_file:
            for var in required_vars_file:
                split_line = var.split()
                # Regular case of variable written straight to the model
                if len(split_line) == 1:
                    if var.strip() not in keys:
                        print(f"Important context is missing: {var}")
                        return chosen_config
                # String case with possible values
                else:
                    if split_line[0] not in keys:
                        print(f"Important context is missing: {split_line[0]}")
                        return chosen_config
                    else:
                        if len(split_line[1:]) <= 1:
                            print(f"Too few possible values for string variable: {split_line[0]}")
                            return chosen_config
                        str_vars[split_line[0]] = split_line[1:]

        # Write qr-metrics and context values to the model (all assumed to be given as doubles)
        # Write state variables (config) to model as initial values (assumed to be given as ints)
        with open(f'{model_dir}/base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()
        for qr in adaptation_state.qrs:
            base_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'
        for kv in adaptation_state.context:
            base_model += f'\nconst double {kv.key.lower()} = {kv.value};'
        for kv in adaptation_state.config:
            base_model += f'\nconst {kv.key.lower()}_init = {kv.value};'

        with open(f'{model_dir}/final_model.pm','w') as model_file:
            model_file.write(base_model)
        
        # Generate optimal strategy optimising over specified property
                
        completed_process = subprocess.run(
            [f'{prism_bin} {model_dir}/final_model.pm -pf \'{prop}\' -exportstrat stdout:reach=false'],
            shell=True, capture_output=True, text=True)
        
        # Parse output to obtain strategy
        output = completed_process.stdout
        # print(output)
        strategy = {}
        strategy_string = output.split("Exporting strategy as actions below:\n")[1].split("\n---")[0]
        for line in strategy_string.splitlines():
            (state, action) = line.split('=')
            # Have state as an array of values
            if action:
                config_index = int(action.split("config")[1])   
                strategy[state] = config_index

        # TODO: check that it's a valid state. Perhaps a similar thing to the context, where a file needs to be provided with all state parameters?
        current_state = '('
        for param in adaptation_state.config:
            current_state += param.value + ','
        current_state = current_state[:-1] + ')'

        print(f"current_state: {current_state}")

        # TODO: There's also a problem where a valid state won't be in the strategy if the goal is already met or it's a deadlock. have to find a solution for it
        if current_state in strategy:
            choice = strategy[current_state]
            chosen_config = possible_configs[choice]
            print(f"chosen_config: {chosen_config}")

        # print(strategy)
        # print(current_state)
        # print(f"choice: {choice}")


        return chosen_config
    