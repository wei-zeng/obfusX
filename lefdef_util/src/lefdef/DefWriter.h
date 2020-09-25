/**
 * @file    DefWriter.h
 * @author  Jinwook Jung (jinwookjung@kaist.ac.kr)
 * @date    2018-10-24 17:12:41
 *
 * Created on Wed Oct 24 17:12:41 2018.
 */
#ifndef DEFWRITER_H
#define DEFWRITER_H

#include "common_header.h"

#include "Lef.h"
#include "Def.h"
#include "util.h"
#include "grDB_DR.h"

class MyDefWriter
{
public:
    static MyDefWriter& get_instance ();

    void write_def (def::Def& def, const RoutingDB_DR &routingDB, string filename);

private:
    def::Def* def_;

    // Do not allow instantiation of this class.
    MyDefWriter ();
    ~MyDefWriter () = default;
    MyDefWriter (const MyDefWriter&) = delete;
    MyDefWriter& operator= (const MyDefWriter&) = delete;
    MyDefWriter (MyDefWriter&&) = delete;
    MyDefWriter& operator= (MyDefWriter&&) = delete;
    static void write_rows (def::Def* def);
    static void write_tracks (def::Def* def);
    static void write_gcell_grids (def::Def* def);
    static void write_components (def::Def* def);
    static void write_pins (def::Def* def);
    static void write_special_nets (def::Def* def);
    static void write_nets (def::Def* def, const RoutingDB_DR &routingDB);
};

#endif /* DEFWRITER_H */
