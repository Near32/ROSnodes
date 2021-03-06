#ifndef RUNNINGSTATS_H
#define RUNNINGSTATS_H

#include <iostream>
#include <fstream>
#include <map>
#include <string>

template<typename T>
class RunningStats
{
	typedef typename std::map<T,T>::iterator statIt;
	typedef typename std::map<string,T>::iterator lstatIt;
	
	typedef typename std::map<T,std::vector<T> >::iterator tStatIt;
	typedef typename std::vector<T>::iterator valIt;
	
	public :
	
	RunningStats(std::string file_) : file(file_), tFile(file_+".t.txt")
	{
		counter = 0;
		valC = 1;
	}
	
	~RunningStats()
	{
	
	}
	
	inline void add(const T stat)
	{
		T val = 1;
		if( stats.count(stat) == 1)
		{
			val += stats[stat];
		}
		
		stats[stat] = val;
		
		this->writeFile();
	}
	
	inline void ladd(const string lstat)
	{
		T val = 1;
		if( lstats.count(lstat) == 1)
		{
			val += lstats[lstat];
		}
		
		lstats[lstat] = val;
		
		this->writeFile();
	}
	
	inline void mean(const T stat, const T val)
	{
		T value = val;
		if( stats.count(stat) == 1)
		{
			value += stats[stat];
		}
		
		stats[stat] = value;
		
		this->writeFile();
	}
	
	inline void lmean(const string lstat, const T val)
	{
		T value = val;
		if( lstats.count(lstat) == 1)
		{
			value += lstats[lstat];
		}
		
		lstats[lstat] = value;
		
		this->writeFile();
	}

	inline void tWriteFile()
	{
		ofstream myfile;
		myfile.open( tFile.c_str() );
		
		tStatIt it;
		
		for(it=tStats.begin();it!=tStats.end();it++)
		{
			myfile << (*it).first << " : " ;
			
			//valIt valit;
			//for(valit = (*it).second.begin() ; valit != (*it).second.end() ; valit++)
			for(int i=0; i<(*it).second.size();i++)
			{
				myfile << (*it).second[i] << " , " ;
			}
			
			myfile << std::endl;
		}
		
		myfile.close();
	}
	
	void writeFile()
	{
		ofstream myfile;
		myfile.open( file.c_str() );
		
		statIt it;
		
		for(it=stats.begin();it!=stats.end();it++)
		{
			myfile << (*it).first << " = " << (*it).second << std::endl;
		}
		
		lstatIt lit;
		for(lit=lstats.begin();lit!=lstats.end();lit++)
		{
			myfile << (*lit).first << " = " << (*lit).second << std::endl;
		}
		
		myfile.close();
	}
	
	inline void tadd(const T tstat, const T val)
	{
		tStats[tstat].insert( tStats[tstat].end(), val);
		
		if(counter > valC)
		{
			this->tWriteFile();
			counter = 0;
		}
		else
		{
			counter++;
		}
	}
	
	
	
	
	private :
	
	int counter;
	int valC;
	std::string file;
	std::string tFile;
	std::map<T,T> stats;
	
	std::map<T,std::vector<T> > tStats;
	//std::map<T,std::vector<T> > stringtStats;
	
	std::map<string,T> lstats;
};

#endif
