/**
 * @file    MyLefDefParser.h
 * @author  Jinwook Jung (jinwookjung@kaist.ac.kr)
 * @date    2019-08-12 23:26:12
 *
 * Created on Tue Oct 16 12:29:08 2018.
 */

#ifndef LefDefParser_H
#define LefDefParser_H

#include "common_header.h"

#include "Lef.h"
#include "Def.h"
#include "util.h"

class MyLefDefParser
{
public:
	void read_lef (string filename);
    void read_def (string filename);

    void write_bookshelf (string filename) const;
    void write_bookshelf_nodes (string filename) const;
    void write_bookshelf_nets (string filename) const;
    void write_bookshelf_wts (string filename) const;
    void write_bookshelf_scl (string filename) const;
    void write_bookshelf_pl (string filename) const;
    void write_bookshelf_route (string filename) const;
    void write_bookshelf_gr (string filename) const;

    void update_def (string bookshelf_pl);

    static MyLefDefParser& get_instance ();

    // Following functions will be removed soon
    def::Def& get_def ();

private:
    lef::Lef&    lef_;
    def::Def&    def_;
    friend class LayoutDR;
    friend class RoutingDB_DR;
    friend class Parser;
    // Do not allow instantiation of this class.
    MyLefDefParser ();
    ~MyLefDefParser () = default;
    MyLefDefParser (const MyLefDefParser&) = delete;
    MyLefDefParser& operator= (const MyLefDefParser&) = delete;
    MyLefDefParser (MyLefDefParser&&) = delete;
    MyLefDefParser& operator= (MyLefDefParser&&) = delete;
};

#endif /* LefDefParser_H */
