import googlemaps
import csv
import json
 
csv_f = open('maha_datafile.csv','r')
f = csv.DictReader(csv_f)
lc=0
nac = 0
gmaps = googlemaps.Client(key='AIzaSyA7B3qfR6viau1pQDKdYHj2-JrDxPRIBPI') 
fr = open('rev_gecode_maha.csv','w')
fr_writer = csv.writer(fr, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
fr_writer.writerow(['h_name','address','city','district','state','lat','lng'])

for row in f:
	if lc == 0:
		pass
	else:
		try:	
			search_str = ''
			if(row['h_name'] != 'NA'):
				search_str += row['h_name'] + ','

			if(row['address'] != 'NA'):
				search_str += row['address'] + ','

			if(row['city'] != 'NA'):
				search_str += row['city'] + ','

			if(row['district'] != 'NA'):
				search_str += row['district'] + ','

			if(row['state'] != 'NA'):
				search_str += row['state']

			geocode_result = gmaps.geocode(search_str)
			val = geocode_result[0]['geometry']['location']
			fr_writer.writerow([row['h_name'], row['address'], row['city'],
							row['district'], row['state'], val['lat'], val['lng']])
			print(geocode_result[0]['geometry']['location'])
		except:
			pass
	lc = lc+1

