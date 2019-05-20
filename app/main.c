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
    0x0f, 0x9a, 0x3e, 0xa6, 0xdd, 0x60, 0xc1, 0xe9, 0x87, 0x89, 0x0c, 0x75, 0x79, 0x68, 0x3d, 0x74, 
    0x40, 0x6c, 0xb4, 0xa7, 0x92, 0xfc, 0xf4, 0xba, 0xc7, 0x69, 0xa7, 0xe2, 0x39, 0x19, 0x2d, 0x03, 
    };

    if (argc >= 2) {
        if (!memcmp("store", argv[1], strlen(argv[1]))) {
            printf("config and store key!\n");
            retval = store_key();
            printf("retval = 0x%x\n", retval);
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

