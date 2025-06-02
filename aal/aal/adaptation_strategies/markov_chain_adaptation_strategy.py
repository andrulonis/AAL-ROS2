#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

# Get path to PRISM binary
prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"

class PrismMarkovChainStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('markov_chain')

    # Store possible values of string parameters in a dictionary
    def handle_string_config_params(self):
        self.str_vars = {}
        if os.path.isfile(f"{self.model_dir}/string_config_params.txt"):
            with open(f'{self.model_dir}/string_config_params.txt', 'r') as string_config_params_file:
                for var in string_config_params_file:
                    split_line = var.split()
                    if len(split_line[1:]) <= 1:
                        print(f"Too few possible values for string variable: {split_line[0]}")
                        return False
                    self.str_vars[split_line[0]] = split_line[1:]
        return True

    # Check if all necessary files are provided
    def check_files_present(self):
        if not os.path.exists(self.model_dir):
            print(f"Path {self.model_dir} does not exist, please provide a correct path.")
            return False
        
        if not os.path.isfile(f"{self.model_dir}/base_model.pm"):
            print(f"File base_model.pm is missing from the given directory.")
            return False
            
        if not os.path.isfile(f"{self.model_dir}/properties.pctl"):
            print(f"File properties.pctl is missing from the given directory.")
            return False
            
        if not os.path.isfile(f"{self.model_dir}/utility_function.py"):
            print(f"File utility_function.py is missing from the given directory.")
            return False
        return True
    
    # Write qr-metrics and context values to the model (all assumed to be given as doubles)
    def write_qr_and_context_to_model(self, qrs, context):
        with open(f'{self.model_dir}/base_model.pm','r') as base_model_file:
            self.final_model = base_model_file.read()
        for qr in qrs:
            self.final_model += f'\nconst double {qr.qr_name.lower()} = {qr.metric};'
        for kv in context:
            self.final_model += f'\nconst double {kv.key.lower()} = {kv.value};'

    # Fill in configuration parameters in model
    def write_config_to_model(self, config_params):
        with open(f'{self.model_dir}/final_model.pm','w') as model_file:
            model_file.write(self.final_model)
            for param in config_params:
                if param.value.type == 1:   # Case of bool
                    model_file.write(f'\nconst bool {param.name} = {str(param.value.bool_value).lower()};')
                elif param.value.type == 2: # Case of integer
                    model_file.write(f'\nconst int {param.name} = {param.value.integer_value};')
                elif param.value.type == 3: # Case of double
                    model_file.write(f'\nconst double {param.name} = {param.value.double_value};')
                elif param.value.type == 4: # Case of string
                    # PRISM does not support strings, so instead write the index of the value
                    if param.name not in self.str_vars or param.value.string_value not in self.str_vars[param.name]:
                        print(f"Value given to string {param.name} not listed as possible value")
                        return False
                    model_file.write(f'\nconst int {param.name} = {self.str_vars[param.name].index(param.value.string_value)};')
                elif param.value.type == 5: # Case of byte array
                    for i, val in enumerate(param.value.byte_array_value):
                        model_file.write(f'\nconst int {param.name}_{i} = {val};')
                elif param.value.type == 6: # Case of bool array
                    for i, val in enumerate(param.value.bool_array_value):
                        model_file.write(f'\nconst bool {param.name}_{i} = {str(val).lower()};')
                elif param.value.type == 7: # Case of integer array
                    for i, val in enumerate(param.value.integer_array_value):
                        model_file.write(f'\nconst int {param.name}_{i} = {val};')
                elif param.value.type == 8: # Case of double array
                    for i, val in enumerate(param.value.double_array_value):
                        model_file.write(f'\nconst double {param.name}_{i} = {val};')
                elif param.value.type == 9: # Case of string array
                    for i, val in enumerate(param.value.string_array_value):
                        # PRISM does not support strings, so instead write the index of the value, with added suffix of "_i", where i is the index within the array received
                        if param.name not in self.str_vars or val not in self.str_vars[param.name]:
                            print(f"Value given to string {param.name}_{i} not listed as possible value")
                            return False
                        model_file.write(f'\nconst int {param.name}_{i} = {self.str_vars[param.name].index(val)};')
        return True

    def suggest_adaptation(self, adaptation_state):
        # Get path to the model, property from the user (from the behaviour tree)
        self.model_dir = adaptation_state.model_dir

        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]
        best_util = float('-inf')

        if not self.check_files_present():
            return chosen_config
            
        if not self.handle_string_config_params():
            return chosen_config

        # Import the utility function from the model directory
        sys.path.append(self.model_dir)
        from utility_function import calculate_utility

        qrs = adaptation_state.qrs
        context = adaptation_state.context

        self.write_qr_and_context_to_model(qrs, context)
        
        for config in possible_configs:
            if not self.write_config_to_model(config.configuration_parameters):
                return chosen_config
                
            # Run PRISM for config
            completed_process = subprocess.run(
                [f'{prism_bin} {self.model_dir}/final_model.pm {self.model_dir}/properties.pctl'],
                shell=True, capture_output=True, text=True)

            # Parse output
            prop_results = []
            output = completed_process.stdout
            results = output.split("Result: ")[1:]
            
            if results == [output]:
                print("PRISM did not give expected output. There may be a syntax error in the model, or some parameters did not get written properly.")
                return chosen_config
            
            for result_string in results:
                value_string = result_string.split()[0]
                # Depending on the property, result may be a bool, int or float
                if value_string == "true":
                    prop_results.append(True)
                elif value_string == "false":
                    prop_results.append(False)
                elif value_string.isdigit():
                    prop_results.append(int(value_string))
                else:
                    prop_results.append(float(value_string))

            # Use provided function to get utility of the adaptation, update chosen config if this one is better
            util = calculate_utility(prop_results)
            if util > best_util:
                chosen_config = config
                best_util = util

        return chosen_config
    