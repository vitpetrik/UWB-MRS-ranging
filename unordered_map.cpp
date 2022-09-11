#include <iostream>
#include <unordered_map>
#include <vector>
#include <zephyr/zephyr.h>
#include <string.h>

typedef std::unordered_map<uint32_t, void *> void_map;

extern "C" void *um_create();
extern "C" void um_add(void_map *map, uint32_t key, void *data);
extern "C" void *um_find(void_map *map, uint32_t key);
extern "C" bool um_contains(void_map *map, uint32_t key);
extern "C" int um_erase(void_map *map, uint32_t key);
extern "C" int um_size(void_map *map);
extern "C" int um_max_size(void_map *map);
extern "C" int um_empty(void_map *map);
extern "C" int um_get_keys(void_map *map, uint32_t *keys);

void *um_create()
{
    void_map *map = new void_map;
    return static_cast<void *>(map);
}

void um_add(void_map *map, uint32_t key, void *data)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    (*the_map)[key] = data;
    // (*the_map).insert({key, data});
}

void *um_find(void_map *map, uint32_t key)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map)[key];
}

bool um_contains(void_map *map, uint32_t key)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map).contains(key);
}

int um_erase(void_map *map, uint32_t key)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map).erase(key);
}
int um_size(void_map *map)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map).size();
}

int um_max_size(void_map *map)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map).max_size();
}

int um_empty(void_map *map)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    return (*the_map).empty();
}

int um_get_keys(void_map *map, uint32_t *keys)
{
    void_map *the_map;
    the_map = static_cast<void_map *>(map);

    int i = 0;
    for (const auto &[key, _] : (*the_map))
    {
        keys[i++] = key;
    }

    return 0;
}