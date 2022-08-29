/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
/*
 * Please do not edit this file.
 * It was generated using rpcgen.
 */

#include <string.h> /* for memset */
#include "nfs.h"

/* This file is copied from RFC1813
 * Copyright 1995 Sun Micrososystems (I assume)
 */

typedef char* caddr_t;

/* Default timeout can be changed using clnt_control() */
static struct timeval TIMEOUT = { 25, 0 };

enum clnt_stat
nfsproc3_null_3(void *clnt_res, CLIENT *clnt)
{
     return (clnt_call(clnt, NFSPROC3_NULL,
        (xdrproc_t) xdr_void, (caddr_t) NULL,
        (xdrproc_t) xdr_void, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_getattr_3(GETATTR3args arg1, GETATTR3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_GETATTR,
        (xdrproc_t) xdr_GETATTR3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_GETATTR3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_setattr_3(SETATTR3args arg1, SETATTR3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_SETATTR,
        (xdrproc_t) xdr_SETATTR3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_SETATTR3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_lookup_3(LOOKUP3args arg1, LOOKUP3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_LOOKUP,
        (xdrproc_t) xdr_LOOKUP3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_LOOKUP3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_access_3(ACCESS3args arg1, ACCESS3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_ACCESS,
        (xdrproc_t) xdr_ACCESS3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_ACCESS3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_readlink_3(READLINK3args arg1, READLINK3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_READLINK,
        (xdrproc_t) xdr_READLINK3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_READLINK3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_read_3(READ3args arg1, READ3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_READ,
        (xdrproc_t) xdr_READ3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_READ3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_write_3(WRITE3args arg1, WRITE3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_WRITE,
        (xdrproc_t) xdr_WRITE3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_WRITE3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_create_3(CREATE3args arg1, CREATE3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_CREATE,
        (xdrproc_t) xdr_CREATE3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_CREATE3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_mkdir_3(MKDIR3args arg1, MKDIR3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_MKDIR,
        (xdrproc_t) xdr_MKDIR3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_MKDIR3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_symlink_3(SYMLINK3args arg1, SYMLINK3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_SYMLINK,
        (xdrproc_t) xdr_SYMLINK3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_SYMLINK3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_mknod_3(MKNOD3args arg1, MKNOD3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_MKNOD,
        (xdrproc_t) xdr_MKNOD3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_MKNOD3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_remove_3(REMOVE3args arg1, REMOVE3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_REMOVE,
        (xdrproc_t) xdr_REMOVE3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_REMOVE3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_rmdir_3(RMDIR3args arg1, RMDIR3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_RMDIR,
        (xdrproc_t) xdr_RMDIR3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_RMDIR3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_rename_3(RENAME3args arg1, RENAME3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_RENAME,
        (xdrproc_t) xdr_RENAME3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_RENAME3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_link_3(LINK3args arg1, LINK3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_LINK,
        (xdrproc_t) xdr_LINK3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_LINK3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_readdir_3(READDIR3args arg1, READDIR3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_READDIR,
        (xdrproc_t) xdr_READDIR3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_READDIR3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_readdirplus_3(READDIRPLUS3args arg1, READDIRPLUS3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_READDIRPLUS,
        (xdrproc_t) xdr_READDIRPLUS3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_READDIRPLUS3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_fsstat_3(FSSTAT3args arg1, FSSTAT3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_FSSTAT,
        (xdrproc_t) xdr_FSSTAT3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_FSSTAT3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_fsinfo_3(FSINFO3args arg1, FSINFO3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_FSINFO,
        (xdrproc_t) xdr_FSINFO3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_FSINFO3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_pathconf_3(PATHCONF3args arg1, PATHCONF3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_PATHCONF,
        (xdrproc_t) xdr_PATHCONF3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_PATHCONF3res, (caddr_t) clnt_res,
        TIMEOUT));
}

enum clnt_stat
nfsproc3_commit_3(COMMIT3args arg1, COMMIT3res *clnt_res, CLIENT *clnt)
{
    return (clnt_call(clnt, NFSPROC3_COMMIT,
        (xdrproc_t) xdr_COMMIT3args, (caddr_t) &arg1,
        (xdrproc_t) xdr_COMMIT3res, (caddr_t) clnt_res,
        TIMEOUT));
}
