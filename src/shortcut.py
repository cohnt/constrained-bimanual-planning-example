import numpy as np
import networkx as nx
from tqdm.auto import tqdm

def shortcut(path, ValidityChecker, num_tries=100, Distance=None, check_size=1e-1):
	if Distance is None:
		Distance = lambda x, y : np.linalg.norm(x - y)
	if type(path) != list:
		path = list(path)
	total_shortcuts = 0

	segment_lengths = [Distance(path[i+1], path[i]) for i in range(len(path) - 1)]
	total_distance = np.sum(segment_lengths)
	weights = segment_lengths / total_distance
	for _ in tqdm(range(int(num_tries))):
		start, stop = np.random.choice(len(path) - 1, size=2, replace=False, p=weights)
		if stop < start:
			start, stop = stop, start
		s, t = np.random.random(2)
		q1 = s * path[start] + (1 - s) * path[start+1]
		q2 = s * path[stop] + (1 - s) * path[stop+1]
		step = q2 - q1
		dist = Distance(q1, q2)
		unit_step = step / dist
		validity_step = check_size * unit_step
		add = True
		for step_count in np.arange(1, 1+int(dist / check_size)):
			if not ValidityChecker(q1 + validity_step * step_count):
				add = False
				break
		if add:
			del path[start+1:stop+1]
			path.insert(start+1, q1)
			path.insert(start+2, q2)

			total_distance -= np.sum(segment_lengths[start:stop+1])
			del segment_lengths[start:stop+1]

			dist = Distance(path[start], q1)
			segment_lengths.insert(start, dist)
			total_distance += dist

			dist = Distance(q1, q2)
			segment_lengths.insert(start+1, dist)
			total_distance += dist

			dist = Distance(q2, path[start+3])
			segment_lengths.insert(start+2, dist)
			total_distance += dist

			weights = segment_lengths / total_distance
			total_shortcuts += 1

	print("Applied %d shortcuts" % total_shortcuts)
	return path