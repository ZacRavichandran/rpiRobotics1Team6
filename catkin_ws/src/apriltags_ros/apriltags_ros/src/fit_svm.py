from sklearn import svm
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_validate


def load_in_data(file, is_stop_signs = False):
	data = []
	labels = []
	with open(file, 'r') as f:
		for line in f:
			data_point = line.replace(" ", "").split(",")
			point = [float(feature.strip("\n")) for feature in data_point[1:]]
			data.append(point)
			labels.append(int(data_point[0]) if not is_stop_signs else 1)

	d= np.array(data)
	y = np.array(labels)
	return d, y

def cross_val(d, y):
	d_train, d_test, y_train, y_test = train_test_split(d, y)
	
	gamma = np.arange(0.0001, 0.001, 0.0001)
	print(gamma)
	c_vals = np.arange(2.5, 3.25, 0.25)
	high_g = 0
	high_c = 0
	high = 0
	splits = min(np.count_nonzero(y == 1), np.count_nonzero(y == 0))
	for c in c_vals:
		for g in gamma:
			clf = svm.SVC(kernel='rbf', C=c, gamma=g)
			score = cross_validate(clf, d, y, cv=splits)
			print("C: %f gamma: %f: %f" %(c, g, np.mean(score['test_score'])))
			if np.mean(score['test_score']) > high:
				high = np.mean(score['test_score'])
				high_g = g 
				high_c = c
	print("high c %f, g %f a %f" %(high_c, high_g, high))
	return svm.SVC(kernel='rbf', C=high_c, gamma=high_g).fit(d, y)

def get_fit_svm():
	d, y = get_data()
	c = 2.750000
	gamma = 0.000290
	return svm.SVC(kernel='rbf', C=c, gamma=g).fit(d, y)

def get_data():
	d_signs, y_signs = load_in_data('stop_sign_data_2.txt', is_stop_signs = True)
	d_noise, y_noise = load_in_data('not_stop_sign_data_2.txt')

	d = np.concatenate([d_signs, d_noise], axis=0)
	y = np.concatenate([y_signs, y_noise], axis=0)
	return d, y	

def main():
	d_signs, y_signs = load_in_data('stop_sign_data.txt', is_stop_signs = True)
	d_noise, y_noise = load_in_data('not_stop_sign_data.txt')

	d_signs_2, y_signs_2 = load_in_data('stop_sign_data_2.txt', is_stop_signs = True)
	d_noise_2, y_noise_2 = load_in_data('not_stop_sign_data_2.txt')

	d = np.concatenate([d_signs, d_noise, d_signs_2, d_noise_2], axis=0)
	y = np.concatenate([y_signs, y_noise, y_signs_2, y_noise_2], axis=0)
	cross_val(d,y)

if __name__ == "__main__":
	main()
