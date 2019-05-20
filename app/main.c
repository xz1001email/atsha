#include <unistd.h>
#include <stdio.h>

void print_usge(void)
{
    printf("Usge:\n");
    printf("store               <store key>\n");
    printf("rollkey             <get random key>\n");;
    printf("sn                  <read serial num>\n");;
    printf("test count keyid    <test authencation>\n");;
}

int main(int argc, char **argv)
{
	int retval;
    int count = 1;
    int keyid = 2;
    int i=0;
    int errcnt = 0;

    uint8_t authkey[16][32];
    uint8_t key[32] = {
        0x14, 0x15, 0x63, 0x37, 0x28, 0x45, 0x73, 0x94, 0x51, 0x34,
        0x61, 0x92, 0x79, 0x3b, 0xec, 0xc4, 0x29, 0xfc, 0xdf, 0x7d,
        0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x53, 0x8e,
        0xe1, 0xd3};

    if (argc >= 2) {
        if (!memcmp("store", argv[1], strlen(argv[1]))) {
            printf("config and store key!\n");
            retval = store_key();
            printf("retval = %d\n", retval);
        } else if (!memcmp("rollkey", argv[1], strlen(argv[1]))) {
            printf("roll key!\n");
            roll_key(1);
        } else if (!memcmp("sn", argv[1], strlen(argv[1]))) {
            printf("read sn!\n");
            sha204_read_sn();
        } else if (!memcmp("test", argv[1], strlen(argv[1]))) {
            if (argc >= 3) {
                count = strtol(argv[2], NULL, 10);
                printf("test authentication %d times\n", count);
            }
            if (argc >= 4) {
                keyid = strtol(argv[3], NULL, 10);
                keyid = keyid%16;
                printf("using keyid %d\n", keyid);
            }
            memcpy(&authkey[keyid][0], key, sizeof(key));
            import_key(authkey);
            while (i < count) {
                retval = get_authentication();
                i++;
                if (retval != 0) {
                    errcnt ++;
                    printf("test count %8d/%d err %d\n", i, count, errcnt);
                } else {
                    printf("\rtest count %8d/%d err %d", i, count, errcnt);
                }
                fflush(stdout);
                usleep(10);
            }
            printf("\n\n");
        } else {
            print_usge();
        }
        return 0;
    } else {
        print_usge();
    }
    return 0;
}

