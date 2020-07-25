Index: sys/arm64/intel/firmware.c
===================================================================
--- sys/arm64/intel/firmware.c	(revision 363353)
+++ sys/arm64/intel/firmware.c	(working copy)
@@ -58,7 +58,8 @@
 	 * The firmware node has no property compatible.
 	 * Look for a known child.
 	 */
-	if (!fdt_depth_search_compatible(node, "intel,stratix10-svc", 0))
+	if (!fdt_depth_search_compatible(node, "intel,stratix10-svc", 0) &&
+	    !fdt_depth_search_compatible(node, "xlnx,zynqmp-firmware", 0))
 		return (ENXIO);
 
 	if (!ofw_bus_status_okay(dev))
