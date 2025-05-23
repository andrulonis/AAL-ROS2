#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismMDPStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('mdp')
 
    def suggest_adaptation(self, adaptation_state, **kwargs):
        # Get path to directory containing the model from commandline arguments
        model_dir = kwargs.get('model_dir', None)
        if model_dir != '':
            full_models_path = model_dir
        else:
            #TODO: 1st solution is to receive an absolute path to the model dir (that contains base_model etc.) from adaptation_state
            # Use a default path if none is provided
            models_path = '~/rebet_ws/src/rebet_frog/PRISM_models/mdp'
            full_models_path = os.path.expanduser(models_path)


        # TODO: get property from user, for now assume we want to maximise the first reward up until a user defined goal
        with open(f'{full_models_path}/property.pctl') as property_file:
            prop = property_file.readline()
        
        # Get path to PRISM program
        prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"
        
        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]

        # Write metrics and context to model to obtain up-to-date model

        # Check if all required parametrised PRISM variables are provided (sometimes not the case in first few iterations),
        # and in case of strings, store possible values of the string in a dictionairy
        str_vars = {}
        keys = [kv.key.lower() for kv in adaptation_state.context] + \
               [qr.qr_name.lower() for qr in adaptation_state.qrs] + \
               [param.name for param in possible_configs[0].configuration_parameters]
        
        # TODO: if no required vars then ignore this step
        with open(f'{full_models_path}/required_vars.txt', 'r') as required_vars_file:
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
        with open(f'{full_models_path}/base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()
        for qr in adaptation_state.qrs:
            base_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'
        for kv in adaptation_state.context:
            base_model += f'\nconst double {kv.key.lower()} = {kv.value};'

        with open(f'{full_models_path}/final_model.pm','w') as model_file:
            model_file.write(base_model)
        
        # Generate optimal strategy optimising over specified property
                
        completed_process = subprocess.run(
            [f'{prism_bin} {full_models_path}/final_model.pm -pf \'{prop}\' -exportstrat stdout:reach=false'],
            shell=True, capture_output=True, text=True)
        
        # Parse output to obtain strategy
        output = completed_process.stdout
        print(output)
        strategy = {}
        strategy_string = output.split("Exporting strategy as actions below:\n")[1].split("\n---")[0]
        for line in strategy_string.splitlines():
            (state, action) = line.split('=')
            # Have state as an array of values
            config_index = int(action.split("config")[1])
            strategy[state] = config_index

        print(strategy)

        # TODO: check that it's a valid state. Perhaps a similar thing to the context, where a file needs to be provided with all state parameters?
        current_state = '('
        for param in adaptation_state.config:
            current_state += param.value + ','
        current_state = current_state[:-1] + ')'

        print(f"current_state: {current_state}")

        # TODO: There's also a problem where a valid state won't be in the strategy if the goal is already met or it's a deadlock. have to find a solution for it
        choice = strategy[current_state]
        chosen_config = possible_configs[choice]

        # print(strategy)
        # print(current_state)
        print(f"choice: {choice}")
        print(f"chosen_config: {chosen_config}")


        return chosen_config
    