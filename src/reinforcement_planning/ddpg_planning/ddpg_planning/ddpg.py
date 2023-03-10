import gym
import numpy as np

from ddpg_planning.Agent import Agent

env = gym.make("LunarLander-v2", continuous=True)

agent = Agent(
    alpha=0.000025,
    beta=0.00025,
    input_dims=[8],
    tau=0.001,
    batch_size=64,
    fc1_dims=400,
    fc2_dims=300,
    n_actions=2,
)

agent.load_models()


print(T.cuda.is_available())
np.random.seed(0)

score_history = []
for i in range(1000):
    done = False
    score = 0
    obs = env.reset()[0]
    while not done:
        act = agent.choose_action(obs)
        new_state, reward, done, info, _ = env.step(act)
        agent.remember(obs, act, reward, new_state, int(done))
        agent.learn()
        score += reward
        obs = new_state

    score_history.append(score)
    print(
        "episode",
        i,
        "score %.2f" % score,
        "100 game average %.2f" % np.mean(score_history[-100:]),
    )

    if i % 25 == 0:
        agent.save_models()
