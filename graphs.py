#!/usr/bin/env python3

from pathlib import Path

import pandas


PATH = Path('data')

# data = {}
# for logs in PATH.glob('*.txt'):
    # with logs.open() as f:
        # data[logs.stem] = [int(i) for i in f.readlines()]

# df = pandas.DataFrame(data)

# datasets = [f.stem.split('_') for f in PATH.glob('*.txt')]
# hostnames, libs, algos, models = (set(d[i] for d in datasets) for i in range(4))


if __name__ == '__main__':
    data_2 = []
    for log in PATH.glob('*.txt'):
        with log.open() as f:
            for line in f.readlines():
                data_2.append(log.stem.split('_') + [int(line)])
    df_2 = pandas.DataFrame(data_2, columns=['hostname', 'lib', 'algo', 'model', 'time'])
    df_2.loc[df_2['lib'] == 'Pinocchio'].groupby('algo').boxplot(by=['model'], sym='')
    # df_2.loc[df_2['model'] == 'lwr']].groupby('algo').boxplot(by=['lib', 'model'], sym='')
