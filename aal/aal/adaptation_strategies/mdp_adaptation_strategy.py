#!/usr/bin/env python3
import subprocess
import os
import sys

from aal.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class PrismMDPStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('prism_mdp')
 
    def suggest_adaptation(self, adaptation_state, **kwargs):
        # Get path to directory containing the model from commandline arguments
        model_dir = kwargs.get('model_dir', None)
        if model_dir != '':
            full_models_path = model_dir
        else:
            # Use a default path if none is provided
            models_path = '~/rebet_ws/src/rebet_frog/PRISM_models'
            full_models_path = os.path.expanduser(models_path)
        
        # Get path to PRISM program
        prism_bin = "~/rebet_ws/src/aal/prism-4.8.1-linux64-x86/bin/prism"
        
        # Initially set chosen configuration to the first one provided
        possible_configs = adaptation_state.possible_configurations
        chosen_config = possible_configs[0]

        # TODO: Check that all actions in the model correspond to possible configurations

        with open(f'{full_models_path}/mdp_base_model.pm','r') as base_model_file:
            base_model = base_model_file.read()
            
        with open(f'{full_models_path}/mdp_final_model.pm','w') as model_file:
            model_file.write(base_model)
        
        with open(f'{full_models_path}/mdp_properties.pctl','r') as properties_file:
            for line in properties_file:
                #TODO: Figure out what reward to optimize, or if there is some way to combine the rewards (probably external function like utility one). For now an example property is used
                # then the chosen_config should be changed based on that external function
                property='Rmin{"energy"}=? [C<=10]'
                
                # Run PRISM for each property
                #uncomment once we figure out the todo above
                # completed_process = subprocess.run(
                #     [f'{prism_bin} {full_models_path}/final_model.pm -pf {line} -exportstrat stdout'],
                #     shell=True, capture_output=True, text=True)
                
                completed_process = subprocess.run(
                    [f'{prism_bin} {full_models_path}/mdp_final_model.pm -pf {property} -exportstrat stdout'],
                    shell=True, capture_output=True, text=True)
                
                # Parse output to obtain strategy
                output = completed_process.stdout
                strategy = {}
                strategy_string = output.split("Exporting strategy as actions below:")[1].split("\n---")[0]
                for line in strategy_string.splitlines():
                    (state, action) = line.split('=')
                    config_index = int(action.split("action")[0])
                    strategy[state] = config_index

                # TODO:
                # Determine current state
                
                

        return chosen_config
    