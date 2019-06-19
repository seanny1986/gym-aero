"""
    This script contains training parameters for the policy search algorithms defined in policies.
    Each algorithm has its own training methodology, and thus its own set of parameters. If you
    implement a new policy search algorithm, you will need to put the policy architecture in
    the policies folder, and build a training wrapper in this folder. Then you will need to add
    the training parameters to this script so that they can be imported into your experiment script.

    -- Sean Morrison, 2018
"""

exp = {
        "env": "GeneralSpline-v0",
        "algs": ["trpo-peb"],
        }

cem = {
        "hidden_dim": 128,
        "iterations": 5000,
        "gamma": 0.99,
        "lr": 1e-4,
        "seed": 343,
        "log_interval": 10,
        "pop_size": 32,
        "elite_frac": 0.2,
        "sigma": 0.2,
        "render": False,
        "save": True,
        "cuda": False,
        "logging": True
        }

ddpg = {
        "network_settings": {
                                "gamma": 0.99,
                                "tau": 0.01
                                },
        "hidden_dim": 128,
        "iterations": 5000,
        "mem_len": 1000000,
        "actor_lr": 1e-4,
        "critic_lr": 1e-4,
        "learning_updates": 1,
        "seed": 343,
        "log_interval": 10,
        "warmup": 100,
        "batch_size": 128,
        "ou_scale": 0.75,
        "ou_mu": 0.75,
        "ou_sigma": 0.15,
        "render": False,
        "save": True,
        "cuda": True,
        "logging": True
        }

gae = {
        "network_settings": {
                                "gamma": 0.99,
                                "lambda": 0.92
                                },
        "hidden_dim": 128,
        "iterations": 5000,
        "batch_size": 256,
        "epochs": 2,
        "lr": 1e-4,
        "seed": 343,
        "log_interval": 10,
        "render": True,
        "save": True,
        "cuda": False,
        "logging": True
        }

ppo = {
        "network_settings": {
                                "gamma": 0.99,
                                "lambda": 0.92,
                                "eps": 0.1
                                },
        "hidden_dim": 256,
        "iterations": 5000,
        "batch_size": 1024,
        "epochs": 4,
        "lr": 1e-4,
        "seed": 343,
        "log_interval": 10,
        "render": False,
        "save": True,
        "cuda": True,
        "logging": True
        }

trpo = {
        "network_settings": {
                                "gamma": 0.99,
                                "tau": 0.97,
                                "max_kl": 1e-2,
                                "damping": 1e-1
                                },
        "hidden_dim": 256,
        "iterations": 5000,
        "log_interval": 10,
        "batch_size": 1024,
        "seed": 343,
        "render": True,
        "save": False,
        "cuda": True,
        "logging": False
        }
