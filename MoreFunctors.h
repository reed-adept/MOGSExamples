#ifndef MOREFUNCTORS_HH
#define MOREFUNCTORS_HH

#include "Aria.h"

// adapters between a function with no return value and ArRetFunctor. A fixed
// value given in the constructor is returned by invokeR(). A version of this
// may be added to ARIA someday.
template<typename Ret>
class GlobalFixedRetFunctor: public virtual ArRetFunctor<Ret>
{
  void (*myFunc)();
  Ret myFixedReturnValue;
private:
  GlobalFixedRetFunctor(void (*func)()) {} // prevent use of this ctor, return value must be set
public:
  GlobalFixedRetFunctor(void (*func)(), Ret returnValue) :
    myFunc(func),
    myFixedReturnValue(returnValue)
  {}
  virtual Ret invokeR() {
    (*myFunc)();
    return myFixedReturnValue;
  }
  virtual void invoke() {
    (*myFunc)();
  }
};

template<typename Ret, typename T>
class FixedRetFunctorC : public virtual ArRetFunctor<Ret>
{
  T* myObj;
  void (T::*myFunc)();
  Ret myFixedReturnValue;
private:
  FixedRetFunctorC(T *obj, void (T::*func)()) {} // prevent use of this ctor, return value must be set
public:
  FixedRetFunctorC(T* obj, void (T::*func)(), Ret returnValue) :
    myObj(obj),
    myFunc(func),
    myFixedReturnValue(returnValue)
  {}
  virtual Ret invokeR() {
    (myObj->*myFunc)();
    return myFixedReturnValue;
  }
  virtual void invoke() {
    (myObj->*myFunc)();
  }
};

// An ArFunctor that creates a new thread to call the target function/method in.
// May be moved into Aria someday.  We would need to create a new class for
// every functor type however, so maybe not the best choice?  Instead we could
// extend ArASyncTask with an accessor that just returns a plain ArFunctor that
// starts the new thread?
template<typename T>
class ASyncFunctorC : public virtual ArFunctorC<T>, public virtual ArASyncTask
{
private:
  ASyncFunctorC() {} // disabled, must supply target object and function 
public:
  ASyncFunctorC(T *obj, void (T::*func)(void)) : ArFunctorC<T>(obj, func)
  {}
  ASyncFunctorC(T &obj, void (T::*func)(void)) : ArFunctorC<T>(obj, func)
  {}
protected:
  virtual void invoke()
  {
    runAsync();
  }

  virtual void *runThread(void *)
  {
    ArFunctorC<T>::invoke();
    return 0;
  }
};

// An ArFunctor that creates a new thread to call the target function/method in.
// May be moved into Aria someday.
class ASyncGlobalFunctor : public virtual ArGlobalFunctor, public virtual ArASyncTask
{
private:
  ASyncGlobalFunctor() {} // disabled, must supply target function 
public:
  ASyncGlobalFunctor(void (*func)(void)) : ArGlobalFunctor(func)
  {}
protected:
  virtual void invoke() {
    runAsync();
  }

  virtual void *runThread(void *) {
    ArGlobalFunctor::invoke();
    return 0;
  }
};


/*
/// When invoked, call a method on an object and return true if the result
/// equals the desired value.
template <typename ObjT, typename RetT>
class FunctorCheckConstMethodResult : virtual ArRetFunctor<bool>
{
protected:
  RetT myCheckVal;
  ArConstRetFunctorC<RetT, ObjT> myFunc;
public:
  FunctorCheckConstMethodResult(ObjT *obj, RetT (ObjT::*func)(void) const, const RetT& val) :
    myCheckVal(val),
    myFunc(obj, func)
  {}

  virtual bool invokeR()
  {
    return(myFunc.invokeR() == myCheckVal);
  }

  virtual void invoke() 
  { // error
  }
};
*/

#endif
