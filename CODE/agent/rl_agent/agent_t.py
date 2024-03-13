from omegaconf import OmegaConf

path='/home/lzqw/PycharmProject/Carla-RL/CARLA/CODE/agent/rl_agent/roach.yaml'
cfg= OmegaConf.load(path)
cfg=OmegaConf.to_container(cfg)
#{'save_data': False, 'data_save': 'data/seed', 'route_file': 'None',
# 'entry_point': 'rl_birdview.rl_birdview_agent:RlBirdviewAgent', 'wb_run_path': None, 'wb_ckpt_step': None,
# 'env_wrapper': {'entry_point': 'rl_birdview.utils.rl_birdview_wrapper:RlBirdviewWrapper',
# 'kwargs': {'input_states': ['control', 'vel_xy'], 'acc_as_action': True}},
# 'policy': {'entry_point': 'rl_birdview.models.ppo_policy:PpoPolicy',
# 'kwargs': {'policy_head_arch': [256, 256], 'value_head_arch': [256, 256], 'features_extractor_entry_point': 'rl_birdview.models.torch_layers:XtMaCNN', 'features_extractor_kwargs': {'states_neurons': [256, 256]}, 'distribution_entry_point': 'rl_birdview.models.distributions:BetaDistribution', 'distribution_kwargs': {'dist_init': None}}}, 'training': {'entry_point': 'rl_birdview.models.ppo:PPO', 'kwargs': {'learning_rate': 1e-05, 'n_steps_total': 12288, 'batch_size': 256, 'n_epochs': 20, 'gamma': 0.99, 'gae_lambda': 0.9, 'clip_range': 0.2, 'clip_range_vf': None, 'ent_coef': 0.01, 'explore_coef': 0.05, 'vf_coef': 0.5, 'max_grad_norm': 0.5, 'target_kl': 0.01, 'update_adv': False, 'lr_schedule_step': 8}}, 'obs_configs': {'birdview': {'module': 'birdview.chauffeurnet', 'width_in_pixels': 192, 'pixels_ev_to_bottom': 40, 'pixels_per_meter': 5.0, 'history_idx': [-16, -11, -6, -1], 'scale_bbox': True, 'scale_mask_col': 1.0}, 'speed': {'module': 'actor_state.speed'},
# 'control': {'module': 'actor_state.control'}, 'velocity': {'module': 'actor_state.velocity'}}}
print(cfg)