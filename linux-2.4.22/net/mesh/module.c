#include "module.h"
#include "pkthandlers.h"
#include "prochandlers.h"
#include "mainthread.h"

u_int8_t passive = 0;

u_int8_t passive_device;

int is_passive(void)
{
    return passive_device;
}

int mesh_init(void)
{
   if (passive==1)
      passive_device=1;
   else
      passive_device=0;

   if (passive_device) printk("Device is passive\n");

   init_mc_cache();
   init_route_table();
   init_message_queue();
   init_timer_queue();
   init_rrequest_cache();
   init_neighbour_list();
   init_multicast_queue();
   init_multicast_sock();
   init_iflist();
   init_iw_sock();
   init_packetqueue();
   start_main_thread();

   /* Register the packet handlers */
   if (init_pkthandlers()) {
       /* Inform user of the problem */
       printk("Failed to register packet handlers\n");
       /* Bail out */
       goto hook_failed;
   }

   /* Register our /proc entries */
   proc_init();

   return 0;

hook_failed:

   return 1;

}

void mesh_deinit(void)
{
   /* Unregister /proc entries */
   proc_deinit();

   kill_multicast_thread();

   /* Unregister the packet handlers */
   deinit_pkthandlers();

   deinit_packetqueue();
   deinit_message_queue();
   cleanup_rrequest_cache();
   stop_main_thread();
   cleanup_timer_queue();
   cleanup_route_table();
   close_sock();
   close_iw_sock();
}

EXPORT_NO_SYMBOLS;

module_init(mesh_init)
module_exit(mesh_deinit)

MODULE_PARM(passive, "i");

