import csv
from math import sin, cos, sqrt, atan2, radians

csv_f = open('rev_geocode','r')
f = csv.DictReader(csv_f)

comp_field = 'state'
def calc_dist(lat1,lng1,lat2,lng2):
	R = 6373.0

	lat1 = radians(lat1)
	lng1 = radians(lng1)
	lat2 = radians(lat2)
	lng2 = radians(lng2)

	dlon = lng2 - lng1
	dlat = lat2 - lat1

	a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))

	distance = R * c
	return distance
dist_dict = dict()

for rows1 in f:
	if lc != 0:
		for rows2 in f:
			a = dict()
			if lc!=0 and rows2[comp_field] == rows1[comp_field]:
				a[]

