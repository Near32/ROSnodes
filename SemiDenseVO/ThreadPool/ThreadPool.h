#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include "../../MATv2/Mat.h"


std::mutex sync;


template<typename T>
class Param
{
	public :
	
	Mat<T> param;
	void* referencePointer;
	
	
	Param() : param(Mat<T>(1,1)), referencePointer(NULL)
	{
	
	}
	
	Param(const Mat<T>& p, void* ref) : param(p), referencePointer(ref)
	{
	
	}
	
	
	Param( const Param<T>& p) : param(p.param), referencePointer(p.referencePointer)
	{
	
	}
	
	
	~Param()
	{
	
	}
	
};

template<typename T,class User>
using Func = void (User::*)( Param<T>*, bool*);

template<typename T,class User>
class ThreadPool
{
	private :
	
	bool stop;
	int poolSize;
	
	std::vector<Func<T,User> > FuncQueue;
	std::vector<Param<T>* > ParamQueue;
	
	std::vector<Func<T,User> > completedFuncQueue;
	std::vector<Param<T>* > completedParamQueue;
	//let us not store all the data but only the last one to be in the pool.
	
	std::vector<Func<T,User> > runningFuncQueue;
	std::vector<Param<T>* > runningParamQueue;
	std::vector<bool*> finishedQueue;
	
	
	std::thread* managerLoop;
	std::vector<std::thread* > threads;
	
	//-------------------------------------------//
	//-------------------------------------------//
	//-------------------------------------------//
	//-------------------------------------------//
	
	
	public :
	
	
	ThreadPool(const int poolSize_ = 4) : stop(false),poolSize(poolSize_)
	{
		//managerLoop = new std::thread( &ThreadPool::manager, std::ref(this) );
		managerLoop = new std::thread( &ThreadPool::manager, this );
		//let us start the manager.
	}
	
	
	~ThreadPool()
	{
		this->stop = true;
		
		managerLoop->detach();
		while( managerLoop->joinable() );		
		delete managerLoop;
		
		for(int i=threads.size();i--;)
		{
			threads[i]->detach();
			while(threads[i]->joinable() );
			delete threads[i];
		}
		
		for(int i=completedParamQueue.size();i--;)
		{
			delete completedParamQueue[i];
		}
		//TODO : let us assert that it is okay to handle the pool like this...?
		
	}
	
	
	
	void manager()
	{
		while( !this->stop )
		{
			for(int i=0;i<poolSize;i++)
			{
				sync.lock();
				if(FuncQueue.size() > 0)
				{
					/* let us make sure that we have initialize all of our thread that are in the pool : */
					int size = this->threads.size();
					//we do not enter in this loop if all of our thread are initialized in the pool
					for(int ii=size;ii<poolSize;ii++)
					{
						//this->threads.insert( this->threads.begin(), new std::thread( FuncQueue[0], ParamQueue[0].referencePointer, ParamQueue[0].param ) );
						//this->threads.insert( this->threads.begin(), new std::thread( FuncQueue[0], std::ref(ParamQueue[0].referencePointer), (Param<T>*)( &(ParamQueue[0]) ) ) );
						finishedQueue.insert( finishedQueue.begin(), new bool(false) );
						this->threads.insert( this->threads.begin(), new std::thread( (Func<T,User>)(FuncQueue[0]), ( (User*)(ParamQueue[0]->referencePointer) ), (Param<T>*)(ParamQueue[0]), finishedQueue[0] ) );
					
						transferRunningTaskQueue();
					}
					
					/* let us manage all of our running thread and replace them if they are finished : */
					for(int ii=0;ii<size;ii++)
					{
						if( !(*finishedQueue[ii]) )//threads[ii]->joinable() )
						{
							//then it is running, nothing to do...
							//unless we want to stop...
							//TODO : take care of the stopping of the running threads...
						}
						else
						{
							//then it is no longer running, we can delete it :
							int idx = ii;
							
							//deletation of the thread :
							threads[idx]->detach();
							delete threads[idx];
							threads.erase( threads.begin()+idx );
							
							delete finishedQueue[idx];
							finishedQueue.erase(finishedQueue.begin()+idx);
							
							//handling arguments of the thread :
							transferCompletedTaskQueue(idx);
							
							//loop issue :
							ii--;
						}
					}
					
					sync.unlock();
				}
				else
				{
					sync.unlock();
					break;
				}
				
			}
		}
	}
	
	void transferRunningTaskQueue()
	{
		runningFuncQueue.insert( runningFuncQueue.end(), FuncQueue[0] );
		FuncQueue.erase(FuncQueue.begin()+0);
		
		runningParamQueue.insert( runningParamQueue.end(), ParamQueue[0] );
		ParamQueue.erase(ParamQueue.begin()+0);
	}
	
	void transferCompletedTaskQueue(int idx)
	{
		completedFuncQueue.insert( completedFuncQueue.end(), runningFuncQueue[idx] );
		runningFuncQueue.erase(runningFuncQueue.begin()+idx);
		
		completedParamQueue.insert( completedParamQueue.end(), runningParamQueue[idx] );
		runningParamQueue.erase(runningParamQueue.begin()+idx);
		
		if(completedParamQueue.size() > poolSize)
		{
			completedParamQueue.erase(completedParamQueue.begin());
			//let us not store all the data but only the last one to be in the pool.
		}
	}
	
	
	void reduce( const Func<T,User>& FuncPointer, User* referencePointer,const Param<T>& paramBegin, const Param<T>& paramStep, const Param<T>& paramEnd)
	{
		Mat<T> param(paramBegin.param);
		Mat<T> step(paramStep.param);
		Mat<T> end(paramEnd.param);
		
		sync.lock();
		while( param <= end)
		{
			FuncQueue.insert( FuncQueue.end(), FuncPointer);
			ParamQueue.insert( ParamQueue.end(), new Param<T>(param, (void*)referencePointer) );
			
			param = param+step;
			
		}
		sync.unlock();
		
	}   
	
	
};


#endif
