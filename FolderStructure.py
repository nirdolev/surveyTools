from os import mkdir, rename
from os.path import join
import sys
import os
from glob import glob
from pathlib import Path

import pdb


def fileparts(file_path):
    p = Path(file_path)
    return [p.parent,p.name,p.suffix]


def isEqualNames(name1,name2):
    '''checks if a name is equal to another, considering capital letters'''
    if(name1==name2):
        return 1
    elif(name1.lower()==name2.lower()):
        return 2
    else:
        return 0

def Fix2Capital(dir_name):
    '''changes first letter of name to capital'''
    return dir_name[0].upper() + dir_name[1:]
        

def fast_scandir(dirname):
    '''recurssive subfolder output'''
    subfolders= [f.path for f in os.scandir(dirname) if f.is_dir()]
    for dirname in list(subfolders):
        subfolders.extend(fast_scandir(dirname))
    return subfolders

def FixFolderStructureNames(destination_folder,benchmark_dir_lst):
    '''changes folders name in benchmark_dir_lst that exist in destination_folder to be first letter capital'''
    #get all sub folders in destination_folder
    all_dirs=fast_scandir(destination_folder)

    for i in range(0,len(all_dirs)):
        for j in range(0,len(benchmark_dir_lst)):
            if(isEqualNames(all_dirs[i],benchmark_dir_lst[j])==2):
                os.rename(all_dirs[i],benchmark_dir_lst[j])


def CreateFolderStructure(benchmark_dir_lst):
    '''creates the folders in benchmark_dir_lst'''
    for folder in benchmark_dir_lst:
        if(not os.path.exists(folder)):
            mkdir(folder)


def ReportFolderStructureExist(destination_folder,benchmark_dir_lst):
    '''prints report to stdout if the folders in benchmark_dir_lst exist in destination_folder'''
    for folder in benchmark_dir_lst:
        name=os.path.dirname(folder)
        if(os.path.exists(folder)):
            print("%s YES" % (name))
        else:
            print("%s NO" % (name))
    

def ReportFolderStructureExact(destination_folder,benchmark_dir_lst):
    '''prints report to stdout if destination_folder contains other folder beside the folders in benchmark_dir_lst 
    or if some folders in benchmark_dir_lst does not exist in destination_folder'''

    #get all sub folders in destination_folder
    all_dirs=fast_scandir(destination_folder)
    for f in all_dirs:
        if(not f.lower() in (string.lower() for string in benchmark_dir_lst)): 
            print("%s is off structure" % (f))

    for folder in benchmark_dir_lst:
        if(not os.path.exists(folder)):
            print("%s Not Exist" % (os.path.dirname(folder)))


def FixDestinationFolderNames(destination_folder, is_deep):
    '''change all folders names in destination_folder folder to be the first letter capital'''
    
    if(is_deep):
        folders=fast_scandir(destination_folder)
    else:
        folders=[os.path.join(destination_folder,f) for f in os.listdir(destination_folder) if os.path.isdir(os.path.join(destination_folder,f)) ]
        
    for f in folders:
        [dir_path,name,ext]=fileparts(f)
        first_letter_ascii=ord(name[0])
        if(97<=first_letter_ascii<=122): #lowercase first letter
            new_f=os.path.join(dir_path,Fix2Capital(name))
            os.rename(f,new_f)


if __name__=="__main__":
    if(len(sys.argv)<3 or len(sys.argv)>4):
        print("usage1: pyhton FolderStructure.py destination_folder --fix/--fix_deep")
        print("usage1: pyhton FolderStructure.py destination_folder folders_file.txt --create\--exist\--exact\--fix")
        sys.exit(1)
    
    destination_folder=sys.argv[1]
    
    if(len(sys.argv)==3):
        flag=sys.argv[2]

        if(flag=='--fix'):
            FixDestinationFolderNames(destination_folder, False)
        elif(flag=='--fix_deep'):
            FixDestinationFolderNames(destination_folder, True)
        else:
            print("wrong flag.\nflag options: --fix \ --fix_deep")
            sys.exit(1)


    if(len(sys.argv)==4):
        folders_file=sys.argv[2]
        flag=sys.argv[3]
    
        try:
            
            #read folder file 
            with open(folders_file,'r') as ff: 
                all_lines = ff.read()
                dir_lst=all_lines.split('\n') #holds the list of folders
                dir_lst=[f for f in dir_lst if(not f=='')] #clean from empty newlines
                benchmark_dir_lst=[destination_folder + "\\" + f for f in dir_lst]
                
                #create folders
                if(flag=='--create'):
                    CreateFolderStructure(benchmark_dir_lst)
                    FixFolderStructureNames(destination_folder,benchmark_dir_lst)

                #check if folders exist
                #not sensitive to case
                elif(flag=='--exist'):
                    ReportFolderStructureExist(destination_folder,benchmark_dir_lst)

                #check if folders exist and no other folders exist but them
                #not sensitive to case
                elif(flag=='--exact'):   
                    ReportFolderStructureExact(destination_folder,benchmark_dir_lst)

                elif(flag=='--fix'):   
                    FixFolderStructureNames(destination_folder,benchmark_dir_lst)

                #handle wrong usage
                else:
                    print("wrong flag.\nflag options: --create \ --exist \ --exact")
                    sys.exit(1)

        #handle exceptions
        except IOError:
            print("can't open folder file!!!")
        except Exception:
            print("unknown error!!!")
            
    sys.exit(0)


    