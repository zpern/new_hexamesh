#ifndef _SINGLETON_H_
#define _SINGLETON_H_
#define _CRT_NO_TIME_T
#include <cassert>

template<typename T> class Singleton
{
protected:
    static T* ms_Singleton;

protected:
    Singleton( void )
    {
        assert( !ms_Singleton );
        ms_Singleton = static_cast<T*>(this);
    }
public:
    ~Singleton( void )
    {  assert( ms_Singleton );  ms_Singleton = 0;  }

    static T& GetSingleton( void )
    {  return *(instance());  }

    static T* instance( void )
    {
        if(!ms_Singleton)
        {
            ms_Singleton = new T();
        }
        return ms_Singleton;
    }
};

template<class T> T* Singleton<T>::ms_Singleton = 0;

#endif

