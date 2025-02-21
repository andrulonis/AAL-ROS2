from aal.adaptation_strategies.random_adaptation_strategy import RandomStrategy
from aal.adaptation_strategies.bandit_adaptation_strategy import BanditStrategy
from aal.adaptation_strategies.prism_adaptation_strategy import PrismStrategy


def create_strategy(strategy_name):
    online_strategies = {
        "random_strategy": RandomStrategy,
        "ucb_strategy": (BanditStrategy, "UCB"),
        "prism_strategy": PrismStrategy
    }

    
    chosen_strategy = online_strategies.get(strategy_name, None)

    if(chosen_strategy is not None):
        if(type(chosen_strategy) is tuple): return chosen_strategy[0](*chosen_strategy[1:])
        else: return chosen_strategy()
    else:
        raise RuntimeError("Specified Strategy " + strategy_name + " did not exist")
