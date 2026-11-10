#include <cassert>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#define ESP32
#define RATGDO_HTTP_LIFECYCLE_EXPERIMENT
#define SSE_MAX_CHANNELS 8
#define ESP_LOGD(tag, format, value) ((void)(value))
constexpr unsigned INADDR_NONE=0;
static unsigned closed;
struct Handle {
    int fd;
    explicit Handle(int n):fd(n){}
    void close(){if(fd>=0){++closed;fd=-1;}}
    ~Handle(){close();}
};
struct WiFiClient {
    std::shared_ptr<Handle> handle;
    WiFiClient()=default;
    explicit WiFiClient(int fd):handle(std::make_shared<Handle>(fd)){}
    int fd()const{return handle?handle->fd:-1;}
    void stop(){if(handle)handle->close();handle.reset();}
};
using String=std::string;
using IPAddress=unsigned;
struct Ticker {void detach(){}};
using SemaphoreHandle_t=std::mutex*;
constexpr int pdTRUE=1,portMAX_DELAY=-1;
static bool xSemaphoreTake(std::mutex*m,int wait){if(wait==0)return m->try_lock();m->lock();return true;}
static void xSemaphoreGive(std::mutex*m){m->unlock();}
enum class HttpOwnedState {Empty,Active,Release,Aborted,Quarantined};
struct HttpOwnedResponse {
    WiFiClient owner;
    int fd=-1;
    uint32_t generation=0,lastProgress=0;
    bool discardQueued=false;
    HttpOwnedState state=HttpOwnedState::Empty;
};
static HttpOwnedResponse httpOwned[2];
static struct {unsigned tracked=0;}httpOwnedCounters;
static bool httpOwnedPending;
static uint32_t _millis(){return 100;}
// SOURCE_HTTP_ROOM
// SOURCE_SSE_HELPERS
// SOURCE_SSE_REMOVE

int main(){
    std::mutex mutex;sseMutex=&mutex;
    for(unsigned i=0;i<2;++i){httpOwned[i].owner=WiFiClient(100+i);httpOwned[i].state=HttpOwnedState::Active;}
    for(unsigned i=0;i<8;++i){
        auto&s=subscription[i];s.client=WiFiClient(200+i);s.clientIP=1;
        s.clientUUID="client"+std::to_string(i);s.active=true;s.SSEconnected=true;
        ++subscriptionCount;
    }
    firmwareUpdateSub=&subscription[0];
    for(auto&s:subscription){SSELock lock;removeSSEsubscription(&s);removeSSEsubscription(&s);}
    assert(subscriptionCount==0&&firmwareUpdateSub==nullptr&&closed==0);
    for(unsigned i=0;i<8;++i)assert(subscription[i].retiring&&retainedBySSE(200+i));
    httpOwnedPending=true;assert(!drainRetiredSSE());httpOwnedPending=false;
    assert(!drainRetiredSSE()); // Full registry cannot close or evict an SSE owner.
    assert(closed==0&&httpOwnedCounters.tracked==0);
    for(unsigned i=0;i<8;++i)assert(subscription[i].client.fd()==int(200+i));
    httpOwned[0].owner.stop();httpOwned[0]=HttpOwnedResponse{};
    for(unsigned i=0;i<8;++i){
        assert(!drainRetiredSSE()); // One available slot is consumed by handoff.
        assert(httpOwned[0].owner.fd()==int(200+i)&&httpOwned[0].discardQueued);
        assert(!retainedBySSE(200+i)&&!subscription[i].retiring);
        assert(subscription[i].client.fd()==-1&&subscription[i].clientIP==INADDR_NONE);
        assert(subscription[i].clientUUID.empty()&&closed==i+1);
        for(unsigned j=i+1;j<8;++j)assert(subscription[j].retiring&&subscription[j].client.fd()==int(200+j));
        // Simulate completion after core ownership validation/abort.
        httpOwned[0].owner.stop();httpOwned[0]=HttpOwnedResponse{};
    }
    assert(drainRetiredSSE()&&httpOwnedCounters.tracked==8&&closed==9);
    assert(httpOwned[1].owner.fd()==101); // The unrelated ordinary owner survives.
    // An already closed shared handle needs no transfer or second close.
    auto&s=subscription[0];s.client=WiFiClient(300);s.client.stop();s.retiring=true;
    assert(drainRetiredSSE()&&!s.retiring&&httpOwnedCounters.tracked==8&&closed==10);
}
