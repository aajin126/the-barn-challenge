import random
from random import choice
from polynomial_fit import get_random_traj_points
from polynomial_fit import distance, calc_time
import argparse





def create_worlds(num_worlds, min_order, max_order, min_objects, max_objects, min_speed, max_speed, min_std, max_std):
    n = random.randint(min_objects, max_objects)

    for i in range(n):
        while True:
            try:
                order = random.randint(min_order, max_order)
                x, y, points = get_random_traj_points(order=order)
            except Exception:
                continue
            else:
                break

    avg_speed = round(random.uniform(min_speed, max_speed), 1)
    std = round(random.uniform(min_std, max_std), 1)
    times = calc_time(x, y, points, avg_speed=avg_speed, min_speed=min_speed, max_speed=max_speed, std=std)

    result = zip(times,x,y)
    result = [i for i in result]

    return result, times, x, y

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arguments for the worlds')
    parser.add_argument('--num_worlds', type=int, required=True)
    parser.add_argument('--min_order', type=int, required=True)
    parser.add_argument('--max_order', type=int, required=True)
    parser.add_argument('--min_objects', type=int, required=True)
    parser.add_argument('--max_objects', type=int, required=True)
    parser.add_argument('--min_speed', type=float,required=True)
    parser.add_argument('--max_speed', type=float, required=True)
    parser.add_argument('--min_std', type=float,required=True)
    parser.add_argument('--max_std', type=float, required=True)
    args = parser.parse_args()
    results, times, x, y = create_worlds(args.num_worlds, args.min_order, args.max_order, args.min_objects,args.max_objects,args.min_speed,args.max_speed, args.min_std, args.max_std)
    print(results)
    # print('time', times)
    # print('x', x)
    # print('y', y)