from sklearn import svm
import numpy as np
from sklearn.model_selection import train_test_split


def load_in_data(file, is_stop_signs = False):
	data = []
	labels = []
	with open(file, 'r') as f:
		for line in f:
			data_point = line.replace(" ", "").split(",")
			point = [float(feature.strip("\n")) for feature in data_point[1:]]
			if len(point) != 3:
				print(point)
			data.append(point)
			labels.append(int(data_point[0]) if not is_stop_signs else 1)

	d= np.array(data)
	y = np.array(labels)
	return d, y

def cross_val(d, y):
	d_train, d_test, y_train, y_test = train_test_split(d, y)
	
	gamma = [1, 2, 3, 4]
	c_vals = [0.5, 0.75, 1, 1.2, 1.5, 2, 3, 4, 5, 6, 7, 8, 9, 10]
	for c in c_vals:
		for g in gamma:
			clf = svm.SVC(kernel='linear', C=c, gamma=g).fit(d_train, y_train)
			print("C: %f gamma: %f: %f" %(c, g, clf.score(d_test, y_test)))

def main():
	d_signs, y_signs = load_in_data('stop_sign_data.txt', is_stop_signs = True)
	d_noise, y_noise = load_in_data('not_stop_sign_data.txt')

	d = np.concatenate([d_signs, d_noise], axis=0)
	y = np.concatenate([y_signs, y_noise], axis=0)
	cross_val(d,y)

if __name__ == "__main__":
	main()
