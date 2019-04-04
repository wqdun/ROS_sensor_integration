/*----------------------------------------------------------------*/
/* class TestEvent                                                */
/*----------------------------------------------------------------*/
class TestEvent
{
public:
    void test()
    {
        //do somsthing
        //……

        //触发事件
        myEvent.sendEvent(100);
        myEvent.sendEvent(200);
    }
public:
    //定义事件
    Event<bool,int> myEvent;
};