#define _DEFAULT_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct {
  char **items;
  size_t count;
  size_t cap;
} path_list;

typedef struct {
  char path[4096];
  uint64_t size;
  uint64_t atime;
  uint64_t mtime;
  uint32_t mode;
  uint32_t uid;
  uint32_t gid;
  char padding[4060];
} header;
void init_header(header *h) { memset(h, 0, sizeof(header)); }

int mkdir_p(const char *path, mode_t mode) {
  char tmp[4096];
  size_t len = strlen(path);

  if (len == 0 || len >= sizeof(tmp))
    return -1;

  strncpy(tmp, path, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';

  while (len > 1 && tmp[len - 1] == '/')
    tmp[--len] = '\0';

  for (char *p = tmp + 1; *p; p++) {
    if (*p == '/') {
      *p = '\0';
      if (mkdir(tmp, mode) == -1 && errno != EEXIST)
        return -1;
      *p = '/';
    }
  }
  if (mkdir(tmp, mode) == -1 && errno != EEXIST)
    return -1;

  return 0;
}

void paths_init(path_list *l) {
  l->items = NULL;
  l->count = 0;
  l->cap = 0;
}

void paths_add(path_list *l, const char *path) {
  if (l->count == l->cap) {
    l->cap = l->cap ? l->cap * 2 : 16;
    l->items = realloc(l->items, l->cap * sizeof(char *));
    if (!l->items) {
      perror("realloc");
      exit(1);
    }
  }
  l->items[l->count++] = strdup(path);
}

void paths_free(path_list *l) {
  for (size_t i = 0; i < l->count; i++)
    free(l->items[i]);
  free(l->items);
  l->items = NULL;
  l->count = l->cap = 0;
}

typedef struct {
  char *path;
  header meta;
} meta_entry;

typedef struct {
  meta_entry *items;
  size_t count;
  size_t cap;
} meta_list;

void meta_add(meta_list *l, const char *path, const header *meta) {
  if (l->count == l->cap) {
    l->cap = l->cap ? l->cap * 2 : 16;
    l->items = realloc(l->items, l->cap * sizeof(meta_entry));
    if (!l->items) {
      perror("realloc");
      exit(1);
    }
  }
  l->items[l->count].path = strdup(path);
  l->items[l->count].meta = *meta;
  l->count++;
}

void meta_free(meta_list *l) {
  for (size_t i = 0; i < l->count; i++)
    free(l->items[i].path);
  free(l->items);
  l->items = NULL;
  l->count = l->cap = 0;
}

void help(void) {
  printf("Usage: %s [option] <in_path> <out_path>\nOptions:\n  -p\tpack files "
         "to a .zz file\n  -u\tunpack a .zz file to an out directory\n",
         "zz");
}

void walk(const char *base, const char *rel, path_list *out) {
  char full[4096];

  if (rel && rel[0])
    snprintf(full, sizeof(full), "%s/%s", base, rel);
  else
    snprintf(full, sizeof(full), "%s", base);

  DIR *dir = opendir(full);
  if (!dir)
    return;

  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL) {
    if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
      continue;

    char child_rel[4096];
    if (rel && rel[0])
      snprintf(child_rel, sizeof(child_rel), "%s/%s", rel, ent->d_name);
    else
      snprintf(child_rel, sizeof(child_rel), "%s", ent->d_name);

    char child_full[4096];
    snprintf(child_full, sizeof(child_full), "%s/%s", full, ent->d_name);

    struct stat st;
    if (stat(child_full, &st) != 0)
      continue;

    if (S_ISDIR(st.st_mode)) {
      paths_add(out, child_rel);
      walk(base, child_rel, out);
    } else if (S_ISREG(st.st_mode)) {
      paths_add(out, child_rel);
    }
  }
  closedir(dir);
}

void pack(FILE *out_file, path_list *list, const char *root_path) {
  struct stat st_buf;
  for (size_t i = 0; i < list->count; i++) {
    header h;
    init_header(&h);
    char full_path[4096];
    if (list->items[i][0] == '/')
      snprintf(full_path, sizeof(full_path), "%s", list->items[i]);
    else
      snprintf(full_path, sizeof(full_path), "%s/%s", root_path,
               list->items[i]);
    if (stat(full_path, &st_buf) != 0) {
      perror("stat");
      continue;
    }
    snprintf(h.path, sizeof(h.path), "%s", list->items[i]);
    h.size = (uint64_t)st_buf.st_size;
    h.atime = (uint64_t)st_buf.st_atime;
    h.mtime = (uint64_t)st_buf.st_mtime;
    h.mode = (uint32_t)st_buf.st_mode;
    h.uid = (uint32_t)st_buf.st_uid;
    h.gid = (uint32_t)st_buf.st_gid;

    FILE *in_file = NULL;
    if (S_ISREG(st_buf.st_mode)) {
      in_file = fopen(full_path, "rb");
      if (in_file == NULL) {
        perror("fopen");
        continue;
      }
    }

    if (fwrite(&h, sizeof(header), 1, out_file) != 1) {
      perror("fwrite");
      if (in_file)
        fclose(in_file);
      return;
    }

    if (in_file) {
      char buf[512];
      size_t n;
      while ((n = fread(buf, 1, 512, in_file)) > 0) {
        if (fwrite(buf, 1, n, out_file) != n) {
          perror("fwrite");
          fclose(in_file);
          return;
        }
        if (n < 512) {
          char padding[512] = {0};
          if (fwrite(padding, 1, 512 - n, out_file) != 512 - n) {
            perror("fwrite");
            fclose(in_file);
            return;
          }
        }
      }
      if (ferror(in_file))
        perror("fread");
      fclose(in_file);
    }
  }
}

static int path_is_safe(const char *p) {
  if (p[0] == '/')
    return 0;
  for (const char *c = p; *c; c++) {
    if (c[0] == '.' && c[1] == '.' && (c[2] == '\0' || c[2] == '/'))
      return 0;
  }
  return 1;
}

static void restore_meta(const char *dst, const header *h) {
  chmod(dst, h->mode & 07777);
  struct timespec times[2] = {{(time_t)h->atime, 0}, {(time_t)h->mtime, 0}};
  utimensat(AT_FDCWD, dst, times, 0);
  chown(dst, h->uid, h->gid);
}

int unpack(FILE *in_file, const char *out_path) {
  if (out_path == NULL || out_path[0] == '\0') {
    fprintf(stderr, "unpack: invalid output path\n");
    return -1;
  }
  meta_list dirs;
  dirs.count = dirs.cap = 0;
  dirs.items = NULL;
  int ret = -1;
  while (1) {
    header h;
    init_header(&h);
    if (fread(&h, sizeof(header), 1, in_file) == 0) {
      if (feof(in_file))
        break;
      fprintf(stderr, "unpack: read error\n");
      goto out;
    }
    if (!path_is_safe(h.path)) {
      fprintf(stderr, "unpack: unsafe path in archive: %s\n", h.path);
      goto out;
    }
    char dst[8192];
    if (out_path[strlen(out_path) - 1] == '/')
      snprintf(dst, sizeof(dst), "%s%s", out_path, h.path);
    else
      snprintf(dst, sizeof(dst), "%s/%s", out_path, h.path);

    if (S_ISREG((mode_t)h.mode)) {
      char *slash = strrchr(dst, '/');
      if (slash && slash != dst) {
        *slash = '\0';
        mkdir_p(dst, 0777);
        *slash = '/';
      }
      FILE *out_file = fopen(dst, "wb");
      if (out_file == NULL) {
        perror("fopen");
        goto out;
      }
      char buf[512];
      uint64_t remaining = h.size;
      while (remaining > 0) {
        if (fread(buf, 1, 512, in_file) == 0) {
          fprintf(stderr, "unpack: truncated archive\n");
          fclose(out_file);
          goto out;
        }
        size_t to_write = remaining < 512 ? (size_t)remaining : 512;
        if (fwrite(buf, 1, to_write, out_file) != to_write) {
          perror("fwrite");
          fclose(out_file);
          goto out;
        }
        remaining -= to_write;
      }
      fclose(out_file);
      restore_meta(dst, &h);
    } else {
      mkdir_p(dst, 0777);
      meta_add(&dirs, dst, &h);
    }
  }
  for (size_t i = 0; i < dirs.count; i++)
    restore_meta(dirs.items[i].path, &dirs.items[i].meta);
  ret = 0;
out:
  meta_free(&dirs);
  return ret;
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    help();
    return 1;
  }
  if (strcmp(argv[3], "") == 0) {
    help();
    return 1;
  }

  const char *mode = argv[1];
  if (strcmp(mode, "-p") != 0 && strcmp(mode, "-u") != 0) {
    help();
    return 1;
  } else if (strcmp(mode, "-p") == 0) {
    struct stat root_st;
    if (stat(argv[2], &root_st) != 0) {
      perror("stat");
      help();
      return 1;
    }

    path_list l;
    paths_init(&l);

    FILE *out_file = fopen(argv[3], "wb");
    if (out_file == NULL) {
      perror("fopen");
      paths_free(&l);
      return 1;
    }
    if (S_ISDIR(root_st.st_mode)) {
      walk(argv[2], "", &l);
      pack(out_file, &l, argv[2]);
    } else {
      const char *base = strrchr(argv[2], '/');
      paths_add(&l, base ? base + 1 : argv[2]);
      if (base) {
        char rootdir[4096];
        size_t n = (size_t)(base - argv[2]);
        if (n == 0) {
          strcpy(rootdir, "/");
        } else {
          memcpy(rootdir, argv[2], n);
          rootdir[n] = '\0';
        }
        pack(out_file, &l, rootdir);
      } else {
        pack(out_file, &l, "./");
      }
    }
    fclose(out_file);

    paths_free(&l);
  } else {
    struct stat file_stat;
    if (stat(argv[2], &file_stat) != 0) {
      perror("stat");
      help();
      return 1;
    }
    if (!S_ISREG(file_stat.st_mode)) {
      help();
      return 1;
    }
    FILE *in_file = fopen(argv[2], "rb");
    if (in_file == NULL) {
      perror("fopen");
      return 1;
    }
    int ret = unpack(in_file, argv[3]);
    fclose(in_file);
    if (ret != 0)
      return 1;
  }
  return 0;
}
