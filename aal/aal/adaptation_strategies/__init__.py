from aal.adaptation_strategies.random_adaptation_strategy import RandomStrategy
from aal.adaptation_strategies.bandit_adaptation_strategy import BanditStrategy
from aal.adaptation_strategies.markov_chain_adaptation_strategy import PrismMarkovChainStrategy
from aal.adaptation_strategies.mdp_adaptation_strategy import PrismMDPStrategy


def create_strategy(strategy_name):
    online_strategies = {
        "random_strategy": RandomStrategy,
        "ucb_strategy": (BanditStrategy, "UCB"),
        "markov_chain_strategy": PrismMarkovChainStrategy,
        "mdp_strategy": PrismMDPStrategy
    }

    
    chosen_strategy = online_strategies.get(strategy_name, None)

    if(chosen_strategy is not None):
        if(type(chosen_strategy) is tuple): return chosen_strategy[0](*chosen_strategy[1:])
        else: return chosen_strategy()
    else:
        raise RuntimeError("Specified Strategy " + strategy_name + " did not exist")
