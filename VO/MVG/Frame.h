#ifndef FRAME_H
#define FRAME_H

#include "../MAT/Mat.h"



template<typename T>
class Frame : public Mat<T>
{
    protected :
    int channel;

    public :

    Frame() : Mat<T>()
    {
        this->channel = 1;

    }

    Frame(Mat<T> image, int chan = 1) : Mat<T>(image)
    {
        this->channel = chan;
    }

    ~Frame()
    {
	
    }

    inline Mat<T> get(const Mat<T>& p)
    
    {
    	Mat<T> rim( extract(*this, p.get(1,1), p.get(2,1), p.get(1,1), p.get(2,1)) );
    	return rim;
    }
    /*
    //return a vector of the channeled value at coordinate p.
    {
            //if(p.get(3,1) != (T)1 && p.get(3,1) != (T)0)
            //    p = (T)(1.0/p.get(3,1))*p;
            //homogenization...
            T wx2 = p.get(1,1)-floor(p.get(1,1));
            T wx1 = (T)1.0-wx2;
            T wy2 = p.get(2,1)-floor(p.get(2,1));
            T wy1 = (T)1.0-wy2;
            T wr1 = sqrt(wx1*wx1+wy1*wy1);
            T wr2 = sqrt(wx2*wx2+wy2*wy2);
            Mat<T> kernel(2,2);
            kernel.set( wr1, 1,1);
            kernel.set( wx2, 1,2);
            kernel.set( wy2, 2,1);
            kernel.set( wr2, 2,2);
            kernel = ((T)1.0/norme1(kernel))*kernel;

            Mat<T> rim(transpose( extract( *this, p.get(1,1), p.get(2,1)*channel, p.get(1,1), p.get(2,1)*channel+(channel-1)) ) );

            if(channel == 1)
            {
                Mat<T> pp(p);
                if( isnan(p.get(1,1)) || p.get(1,1) < (T)0 || p.get(1,1) > this->m_line)
                {
                    pp.set((T)0,1,1);
                }

                if( isnan(p.get(2,1) ) || p.get(2,1) < (T)0 || p.get(2,1) > this->m_column)
                {
                    pp.set((T)0,2,1);
                }
		
                rim = sum( sum( extract(*this, floor(pp.get(1,1)), floor(pp.get(2,1)), floor(pp.get(1,1))+1, floor(pp.get(2,1))+1 ) % kernel ));
#ifdef verbose_debug
                cout << "KERNEL : " << endl;
                kernel.afficher();
#endif
            }


            return rim;
    }
    */
    
    inline Mat<T> get(Mat<T>* p)
    
    {
    	Mat<T> rim( extract(*this, p->get(1,1), p->get(2,1), p->get(1,1), p->get(2,1)) );
    	//Mat<T> rim( this->mat[((int)p->get(1,1))-1][((int)p->get(2,1))-1], 1,1 );
    	return rim;
    }
    /*
    //return a vector of the channeled value at coordinate p.
    {
            //if(p.get(3,1) != (T)1 && p.get(3,1) != (T)0)
            //    p = (T)(1.0/p.get(3,1))*p;
            //homogenization...
            T wx2 = p->get(1,1)-floor(p->get(1,1));
            T wx1 = (T)1.0-wx2;
            T wy2 = p->get(2,1)-floor(p->get(2,1));
            T wy1 = (T)1.0-wy2;
            T wr1 = sqrt(wx1*wx1+wy1*wy1);
            T wr2 = sqrt(wx2*wx2+wy2*wy2);
            Mat<T> kernel(2,2);
            kernel.set( wr1, 1,1);
            kernel.set( wx2, 1,2);
            kernel.set( wy2, 2,1);
            kernel.set( wr2, 2,2);
            kernel = ((T)1.0/norme1(kernel))*kernel;

            Mat<T> rim(transpose( extract( *this, p->get(1,1), p->get(2,1)*channel, p->get(1,1), p->get(2,1)*channel+(channel-1)) ) );

            if(channel == 1)
            {
                Mat<T> pp(*p);
                
                if( isnan(p->get(1,1)) || p->get(1,1) < (T)0 || p->get(1,1) > this->m_line)
                {
                    pp.set((T)0,1,1);
                }

                if( isnan(p->get(2,1) ) || p->get(2,1) < (T)0 || p->get(2,1) > this->m_column)
                {
                    pp.set((T)0,2,1);
                }
		
                rim = sum( sum( extract(*this, floor(pp.get(1,1)), floor(pp.get(2,1)), floor(pp.get(1,1))+1, floor(pp.get(2,1))+1 ) % kernel ));
#ifdef verbose_debug
                cout << "KERNEL : " << endl;
                kernel.afficher();
#endif
            }


            return rim;
    }
    */
    


    int getChannel() const
    {
        return channel;
    }

    void setChannel(int chan)
    {
        channel = chan;
    }
    
    void computePyramid(int nbrLevel = 4)
    {
    
    
    }
    
    void copyFrame(const Frame<T>& f)
    {
	    this->channel = f.getChannel();
	    this->copy(f);
	   
    }
    Frame<T>& operator=(const Frame<T>& f)
    {
    	if(this != &f)
    	{
		this->~Frame();
		//Frame(f);
		this->copyFrame(f);
		/*on ne peut pas appeller le constructeur de copie...?*/
    	}

	return *this;
    }

};


#endif
