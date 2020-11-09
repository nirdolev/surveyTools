import sys
import os
import pdb


def create(relative_folders_file,destination_folder='./'):
	
	try:
		with open(relative_folders_file,'r') as fh:
			data = fh.read()
			all_folders=data.split()

	except IOError as error: 
		print(error)

	
	
	for fold in all_folders:
		folder_name = destination_folder + "/" + fold
		try:
			os.mkdir(folder_name)
			print("folder '{}' created ".format(folder_name))
		except FileExistsError:
			print("folder {} already exists".format(folder_name))
		


def main():
	"""Creates a folder structure in "destination_folder". 
		argument:
		--------
		1. relative_folders_file = txt file, where a relative folder name is written in each separate line.
		2. Idestination_folder = Where the folder structure is created. If destination_folder not supplied, then pwd is assumed.
		flags:
		-----
		1. --create = create folder structure
		2. --remove = remove empty folder structure. folders with content wont be removed
		3. --force_remove = emove folder structure and all subfolders + files
	"""
	
	if(len(sys.argv)>4 or len(sys.argv)<3 or \
	(sys.argv[1]!='--create' and sys.argv[1]!='--remove' and sys.argv[1]!='--force_remove')):
		print("usage: " + sys.argv[0] + "[--create / --remove / --force_remove]" + " relative_folders_file" + " [destination_folder]" )
		return 1
		
	#create folder structure
	if len(sys.argv)==3 and sys.argv[1]=='--create':
		create(sys.argv[2])
		
	elif len(sys.argv)==4 and sys.argv[1]=='--create':
		create(sys.argv[2],sys.argv[3])
	
	#remove empty folder structure	
	elif len(sys.argv)==3 and sys.argv[1]=='--remove':
		print("tbd")
	elif len(sys.argv)==4 and sys.argv[1]=='--remove':
		print("tbd")
	#remove folder structure and all subfolders + files
	elif len(sys.argv)==3 and sys.argv[1]=='--force_remove':
		print("tbd")
	elif len(sys.argv)==4 and sys.argv[1]=='--force_remove':
		print("tbd")
	return 0

if __name__ == "__main__":
	main()
